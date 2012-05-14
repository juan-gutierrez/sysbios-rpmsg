/*
 * Copyright (c) 2011, Texas Instruments Incorporated
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
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/family/arm/ducati/Core.h>

#include <ti/ipc/MultiProc.h>

#include "InterruptM3.h"

/* Register access method. */
#define REG16(A)   (*(volatile UInt16 *) (A))
#define REG32(A)   (*(volatile UInt32 *) (A))

#define M3INT_MBX               50
#define M3INT                   19

/* Assigned mailboxes */
#define HOST_TO_IPU_MBX		0 /* Rx on IPU from Host */
#define IPU_TO_HOST_MBX     1 /* Tx to Host from IPU */
#define DSP_TO_HOST_MBX     2 /* Tx to Host from DSP */
#define HOST_TO_DSP_MBX     3 /* Rx on DSP from Host */
#define IPU_TO_DSP_MBX		5
#define DSP_TO_IPU_MBX		6

#define MAILBOX_BASEADDR        (0xAA0F4000)

#define MAILBOX_MESSAGE(M)      (MAILBOX_BASEADDR + 0x040 + (0x4 * M))
#define MAILBOX_FIFOSTATUS(M)   (MAILBOX_BASEADDR + 0x080 + (0x4 * M))
#define MAILBOX_STATUS(M)       (MAILBOX_BASEADDR + 0x0C0 + (0x4 * M))
#define MAILBOX_REG_VAL(M)      (0x1 << (2 * M))

#define MAILBOX_IRQSTATUS_CLR_IPU    (MAILBOX_BASEADDR + 0x124)
#define MAILBOX_IRQENABLE_SET_IPU    (MAILBOX_BASEADDR + 0x128)
#define MAILBOX_IRQENABLE_CLR_IPU    (MAILBOX_BASEADDR + 0x12C)

/*
 *  Ducati control register that maintains inter-core interrupt bits.
 *
 *  Using separate core0 and core1 values to do 16-bit reads/writes
 *  because we do not want to overwrite the other cores value.
 */
#define INTERRUPT_CORE_0       (0x40001000)
#define INTERRUPT_CORE_1       (0x40001000 + 2)

static struct FxnTable {
	InterruptFxn    func;
	UArg   arg;
} table[2];

static UInt32 numPlugged = 0;
static Hwi_Handle interruptDspHwi = NULL;

Void InterruptM3_intShmMbxStub(UArg arg);

/*
 *************************************************************************
 *                      Proxy functions
 *************************************************************************
 */
Void InterruptProxy_intEnable(UInt16 remoteProcId)
{
    InterruptM3_intEnable(remoteProcId);
}

Void InterruptProxy_intDisable(UInt16 remoteProcId)
{
    InterruptM3_intDisable(remoteProcId);
}

Void InterruptProxy_intRegister(UInt16 remoteProcId, InterruptFxn fxn, UArg arg)
{
    InterruptM3_intRegister(remoteProcId, fxn, arg);
}

Void InterruptProxy_intSend(UInt16 remoteProcId, UArg arg)
{
    InterruptM3_intSend(remoteProcId, arg);
}

UInt InterruptProxy_intClear(UInt16 remoteProcId)
{
    return InterruptM3_intClear(remoteProcId);
}


/*
 *************************************************************************
 *                      Module functions
 *************************************************************************
 */

/*!
 *  ======== InterruptM3_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptM3_intEnable(UInt16 remoteProcId)
{
    /*
     *  If the remote processor communicates via mailboxes, we should enable
     *  the Mailbox IRQ instead of enabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
	if (remoteProcId == MultiProc_getId("HOST")) {
		REG32(MAILBOX_IRQENABLE_SET_IPU) = MAILBOX_REG_VAL(HOST_TO_IPU_MBX);
	}
	else if (remoteProcId == MultiProc_getId("DSP")) {
		REG32(MAILBOX_IRQENABLE_SET_IPU) = MAILBOX_REG_VAL(DSP_TO_IPU_MBX);
	}
}

/*!
 *  ======== InterruptM3_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptM3_intDisable(UInt16 remoteProcId)
{
    /*
     *  If the remote processor communicates via mailboxes, we should disable
     *  the Mailbox IRQ instead of disabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
	if (remoteProcId == MultiProc_getId("HOST")) {
		REG32(MAILBOX_IRQENABLE_CLR_IPU) = MAILBOX_REG_VAL(HOST_TO_IPU_MBX);
	}
	else if (remoteProcId == MultiProc_getId("DSP")) {
		REG32(MAILBOX_IRQENABLE_CLR_IPU) = MAILBOX_REG_VAL(DSP_TO_IPU_MBX);
	}
}

/*!
 *  ======== InterruptM3_intRegister ========
 */
Void InterruptM3_intRegister(UInt16 remoteProcId,
                                 InterruptFxn func, UArg arg)
{
    Hwi_Params  hwiAttrs;
    UInt        key;
    Int         index;

    if (remoteProcId == MultiProc_getId("DSP")) {
        index = 0;
    }
    else if (remoteProcId == MultiProc_getId("HOST")) {
        index = 1;
    }
    else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
    }

    /* Disable global interrupts */
    key = Hwi_disable();

    table[index].func = func;
    table[index].arg  = arg;

    InterruptM3_intClear(remoteProcId);

    Hwi_Params_init(&hwiAttrs);
    hwiAttrs.maskSetting = Hwi_MaskingOption_LOWER;

	numPlugged++;
	if (numPlugged == 1) {
		interruptDspHwi = Hwi_create(M3INT_MBX,
				   (Hwi_FuncPtr) InterruptM3_intShmMbxStub,
				   &hwiAttrs,
				   NULL);

		/* Interrupt_intEnable won't enable the Hwi */
		Hwi_enableInterrupt(M3INT_MBX);
	}

    /* Enable the mailbox interrupt to the M3 core */
    InterruptM3_intEnable(remoteProcId);

    /* Restore global interrupts */
    Hwi_restore(key);

}

/*!
 *  ======== InterruptM3_intUnregister ========
 */
Void InterruptM3_intUnregister(UInt16 remoteProcId)
{
    Int index;

    if (remoteProcId == MultiProc_getId("DSP")) {
        index = 0;
    }
    else if (remoteProcId == MultiProc_getId("HOST")) {
        index = 1;
    }
    else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
    }

    /* Disable the mailbox interrupt source */
    InterruptM3_intDisable(remoteProcId);

    /* Delete/disable the Hwi */

	numPlugged--;
	if (numPlugged == 0) {
		Hwi_delete(&interruptDspHwi);
	}

    /* Clear the FxnTable entry for the remote processor */
    table[index].func = NULL;
    table[index].arg  = 0;
}

/*!
 *  ======== InterruptM3_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptM3_intSend(UInt16 remoteProcId, UArg arg)
{
    UInt key;

     /*
     *  Before writing to a mailbox, check whehter it already contains a message
     *  If so, then don't write to the mailbox since we want one and only one
     *  message per interrupt.  Disable interrupts between reading
     *  the MSGSTATUS_X register and writing to the mailbox to protect from
     *  another thread doing an intSend at the same time
     */

    if (remoteProcId == MultiProc_getId("DSP")) {
        key = Hwi_disable();
        while(REG32(MAILBOX_FIFOSTATUS(IPU_TO_DSP_MBX)));
        REG32(MAILBOX_MESSAGE(IPU_TO_DSP_MBX)) = arg;
        Hwi_restore(key);
    } else if(remoteProcId == MultiProc_getId("HOST")) {
        key = Hwi_disable();
        while(REG32(MAILBOX_FIFOSTATUS(IPU_TO_HOST_MBX)));
        REG32(MAILBOX_MESSAGE(IPU_TO_HOST_MBX)) = arg;
        Hwi_restore(key);
    } else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
	}
}

/*!
 *  ======== InterruptM3_intClear ========
 *  Clear interrupt
 */
UInt InterruptM3_intClear(UInt16 remoteProcId)
{
    UInt arg;

    if(remoteProcId == MultiProc_getId("HOST")) {
		arg = REG32(MAILBOX_MESSAGE(HOST_TO_IPU_MBX));
		REG32(MAILBOX_IRQSTATUS_CLR_IPU) = MAILBOX_REG_VAL(HOST_TO_IPU_MBX);
	} else if (remoteProcId == MultiProc_getId("DSP")) {
		arg = REG32(MAILBOX_MESSAGE(DSP_TO_IPU_MBX));
		REG32(MAILBOX_IRQSTATUS_CLR_IPU) = MAILBOX_REG_VAL(DSP_TO_IPU_MBX);
    } else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
	}

    return (arg);
}

/*
 *************************************************************************
 *                      Internal functions
 *************************************************************************
 */

/*!
 *  ======== InterruptM3_intShmMbxStub ========
 */
Void InterruptM3_intShmMbxStub(UArg arg)
{
//    InterruptM3_FxnTable *table;
    UInt msg;

    /* Process messages from the DSP  */
    if ((REG32(MAILBOX_IRQENABLE_SET_IPU) & MAILBOX_REG_VAL(DSP_TO_IPU_MBX)) &&
			REG32(MAILBOX_STATUS(DSP_TO_IPU_MBX)) != 0) {
		msg = REG32(MAILBOX_MESSAGE(DSP_TO_IPU_MBX));
		REG32(MAILBOX_IRQSTATUS_CLR_IPU) = MAILBOX_REG_VAL(DSP_TO_IPU_MBX);
        (table[0].func)(msg, table[0].arg);
    }

    /* Process messages from the HOST  */
    if ((REG32(MAILBOX_IRQENABLE_SET_IPU) & MAILBOX_REG_VAL(HOST_TO_IPU_MBX)) &&
			REG32(MAILBOX_STATUS(HOST_TO_IPU_MBX)) != 0) {
		msg = REG32(MAILBOX_MESSAGE(HOST_TO_IPU_MBX));
		REG32(MAILBOX_IRQSTATUS_CLR_IPU) = MAILBOX_REG_VAL(HOST_TO_IPU_MBX);
        (table[1].func)(msg, table[1].arg);
    }
}

