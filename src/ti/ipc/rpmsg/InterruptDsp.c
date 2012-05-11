/*
 * Copyright (c) 2011-2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== Interrupt.c ========
 *  OMAP4430/Ducati Interrupt Manger
 */

#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/family/c64p/tesla/Wugen.h>

#include <ti/ipc/MultiProc.h>

#include "InterruptDsp.h"

/* Register access method. */
#define REG16(A)   (*(volatile UInt16 *) (A))
#define REG32(A)   (*(volatile UInt32 *) (A))

/* NVIC/INTC values */
#define DSPINT  				5
#define DSPEVENTID              55

/* Assigned mailboxes */
#define HOST_TO_IPU_MBX		0 /* Rx on IPU from Host */
#define IPU_TO_HOST_MBX     1 /* Tx to Host from IPU */
#define DSP_TO_HOST_MBX     2 /* Tx to Host from DSP */
#define HOST_TO_DSP_MBX     3 /* Rx on DSP from Host */
#define IPU_TO_DSP_MBX		5
#define DSP_TO_IPU_MBX		6

#define MAILBOX_BASEADDR    (0x4A0F4000)

#define MAILBOX_MESSAGE(M)  (MAILBOX_BASEADDR + 0x040 + (0x4 * M))
#define MAILBOX_STATUS(M)   (MAILBOX_BASEADDR + 0x0C0 + (0x4 * M))
#define MAILBOX_REG_VAL(M)  (0x1 << (2 * M))

#define MAILBOX_IRQSTATUS_CLR_DSP   (MAILBOX_BASEADDR + 0x114)
#define MAILBOX_IRQENABLE_SET_DSP   (MAILBOX_BASEADDR + 0x118)
#define MAILBOX_IRQENABLE_CLR_DSP   (MAILBOX_BASEADDR + 0x11C)


static struct FxnTable {
	InterruptFxn    func;
	UArg   arg;
} table[2];

static UInt32 numPlugged = 0;
static Hwi_Handle interruptDspHwi = NULL;

Void InterruptDsp_intShmStub(UArg arg);


/*
 *************************************************************************
 *                      Proxy functions
 *************************************************************************
 */
Void InterruptProxy_intEnable(UInt16 remoteProcId)
{
    InterruptDsp_intEnable(remoteProcId);
}

Void InterruptProxy_intDisable(UInt16 remoteProcId)
{
    InterruptDsp_intDisable(remoteProcId);
}

Void InterruptProxy_intRegister(UInt16 remoteProcId, InterruptFxn fxn, UArg arg)
{
    InterruptDsp_intRegister(remoteProcId, fxn, arg);
}

Void InterruptProxy_intSend(UInt16 remoteProcId, UArg arg)
{

    InterruptDsp_intSend(remoteProcId, arg);
}

UInt InterruptProxy_intClear(UInt16 remoteProcId)
{
    return InterruptDsp_intClear(remoteProcId);
}

/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*!
 *  ======== InterruptDsp_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptDsp_intEnable(UInt16 remoteProcId)
{
    if (remoteProcId == MultiProc_getId("HOST")) {
        REG32(MAILBOX_IRQENABLE_SET_DSP) = MAILBOX_REG_VAL(HOST_TO_DSP_MBX);
    }
    else if (remoteProcId == MultiProc_getId("CORE0")) {
        REG32(MAILBOX_IRQENABLE_SET_DSP) = MAILBOX_REG_VAL(IPU_TO_DSP_MBX);
    }
}

/*!
 *  ======== InterruptDsp_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptDsp_intDisable(UInt16 remoteProcId)
{
    if (remoteProcId == MultiProc_getId("HOST")) {
        REG32(MAILBOX_IRQENABLE_CLR_DSP) = MAILBOX_REG_VAL(HOST_TO_DSP_MBX);
    }
    else if (remoteProcId == MultiProc_getId("CORE0")) {
        REG32(MAILBOX_IRQENABLE_CLR_DSP) = MAILBOX_REG_VAL(IPU_TO_DSP_MBX);
    }
}

/*!
 *  ======== InterruptDsp_intRegister ========
 */
Void InterruptDsp_intRegister(UInt16 remoteProcId, InterruptFxn func, UArg arg)
{
    UInt        key;
    Int         index;
    Hwi_Params  hwiParams;

    if (remoteProcId == MultiProc_getId("HOST")) {
        index = 0;
    }
    else if (remoteProcId == MultiProc_getId("CORE0")) {
        index = 1;
    }
    else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
    }

    /* Disable global interrupts */
    key = Hwi_disable();

    table[index].func = func;
    table[index].arg  = arg;

    InterruptDsp_intClear(remoteProcId);

    /* Make sure the interrupt only gets plugged once */
    numPlugged++;
    if (numPlugged == 1) {
        Hwi_Params_init(&hwiParams);
        hwiParams.eventId = DSPEVENTID;
        interruptDspHwi = Hwi_create(DSPINT,
                   (Hwi_FuncPtr)InterruptDsp_intShmStub,
                   &hwiParams,
                   NULL);

        /* Enable the interrupt */
        Wugen_enableEvent(DSPEVENTID);
        Hwi_enableInterrupt(DSPINT);
    }

    /* Enable the mailbox interrupt to the DSP */
    InterruptDsp_intEnable(remoteProcId);

    /* Restore global interrupts */
    Hwi_restore(key);
}

/*!
 *  ======== InterruptDsp_intUnregister ========
 */
Void InterruptDsp_intUnregister(UInt16 remoteProcId)
{
    Int         index;
//    InterruptDsp_FxnTable *table;

    if (remoteProcId == MultiProc_getId("HOST")) {
        index = 0;
    }
    else if (remoteProcId == MultiProc_getId("CORE0")) {
        index = 1;
    }
    else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
    }


    /* Disable the mailbox interrupt source */
    InterruptDsp_intDisable(remoteProcId);

    numPlugged--;
    if (numPlugged == 0) {
        /* Delete the Hwi */
        Hwi_delete(&interruptDspHwi);
    }

    table[index].func = NULL;
    table[index].arg  = NULL;
}

/*!
 *  ======== InterruptDsp_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptDsp_intSend(UInt16 remoteProcId,
                          UArg arg)
{
    UInt key;

    /*
     *  Before writing to a mailbox, check whehter it already contains a message
     *  If so, then don't write to the mailbox since we want one and only one
     *  message per interrupt.  Disable interrupts between reading
     *  the MSGSTATUS_X register and writing to the mailbox to protect from
     *  another thread doing an intSend at the same time
     */
    if (remoteProcId == MultiProc_getId("HOST")) {
        /* Using mailbox 0 */
        key = Hwi_disable();
        if (REG32(MAILBOX_STATUS(DSP_TO_HOST_MBX)) == 0) {
            REG32(MAILBOX_MESSAGE(DSP_TO_HOST_MBX)) = arg;
        }
        Hwi_restore(key);
    }
    else if (remoteProcId == MultiProc_getId("CORE0")) {
        /* Using mailbox 1 */
        key = Hwi_disable();
        if (REG32(MAILBOX_STATUS(DSP_TO_IPU_MBX)) == 0) {
            REG32(MAILBOX_MESSAGE(DSP_TO_IPU_MBX)) = arg;
        }
        Hwi_restore(key);
    }
    else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
    }
}


/*!
 *  ======== InterruptDsp_intClear ========
 *  Clear interrupt
 */
UInt InterruptDsp_intClear(UInt16 remoteProcId)
{
    UInt arg;

    if (remoteProcId == MultiProc_getId("HOST")) {
        /* Mailbox 3 */
        arg = REG32(MAILBOX_MESSAGE(HOST_TO_DSP_MBX));
        REG32(MAILBOX_IRQSTATUS_CLR_DSP) = MAILBOX_REG_VAL(HOST_TO_DSP_MBX);
    }
    else if (remoteProcId == MultiProc_getId("CORE0")) {
        /* Mailbox 2 */
        arg = REG32(MAILBOX_MESSAGE(IPU_TO_DSP_MBX));
        REG32(MAILBOX_IRQSTATUS_CLR_DSP) = MAILBOX_REG_VAL(IPU_TO_DSP_MBX);
    }
    else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
    }

    return (arg);
}

/*
 *************************************************************************
 *                      Internals functions
 *************************************************************************
 */

/*!
 *  ======== InterruptDsp_intShmStub ========
 */
Void InterruptDsp_intShmStub(UArg arg)
{
//    InterruptDsp_FxnTable *table;
    UInt msg;

    /* Process messages from  HOST */
    if ((REG32(MAILBOX_IRQENABLE_SET_DSP) & MAILBOX_REG_VAL(HOST_TO_DSP_MBX)) &&
			REG32(MAILBOX_STATUS(HOST_TO_DSP_MBX)) != 0) {
        msg = REG32(MAILBOX_MESSAGE(HOST_TO_DSP_MBX));
        REG32(MAILBOX_IRQSTATUS_CLR_DSP) = MAILBOX_REG_VAL(HOST_TO_DSP_MBX);
        (table[0].func)(msg, table[0].arg);
    }

    /* Process messages from CORE0 */
    if ((REG32(MAILBOX_IRQENABLE_SET_DSP) & MAILBOX_REG_VAL(IPU_TO_DSP_MBX)) &&
			REG32(MAILBOX_STATUS(IPU_TO_DSP_MBX)) != 0) {
        msg = REG32(MAILBOX_MESSAGE(IPU_TO_DSP_MBX));
        REG32(MAILBOX_IRQSTATUS_CLR_DSP) = MAILBOX_REG_VAL(IPU_TO_DSP_MBX);
        (table[1].func)(msg, table[1].arg);
    }
}
