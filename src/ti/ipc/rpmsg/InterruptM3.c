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
#define HOST_TO_SYSM3_MBX       0 /* Rx on SysM3 from Host */
#define M3_TO_HOST_MBX          1 /* Tx to Host from M3 */
#define DSP_TO_HOST_MBX         2 /* Tx to Host from DSP */
#define HOST_TO_DSP_MBX         3 /* Rx on DSP from Host */
#define SYSM3_TO_APPM3_MBX      4 /* Rx on AppM3 from Host/SysM3 */

#define MAILBOX_BASEADDR        (0xAA0F4000)

#define MAILBOX_MESSAGE(M)      (MAILBOX_BASEADDR + 0x040 + (0x4 * M))
#define MAILBOX_FIFOSTATUS(M)   (MAILBOX_BASEADDR + 0x080 + (0x4 * M))
#define MAILBOX_STATUS(M)       (MAILBOX_BASEADDR + 0x0C0 + (0x4 * M))
#define MAILBOX_REG_VAL(M)      (0x1 << (2 * M))

#define MAILBOX_IRQSTATUS_CLR_M3    (MAILBOX_BASEADDR + 0x124)
#define MAILBOX_IRQENABLE_SET_M3    (MAILBOX_BASEADDR + 0x128)
#define MAILBOX_IRQENABLE_CLR_M3    (MAILBOX_BASEADDR + 0x12C)

/*
 *  Ducati control register that maintains inter-core interrupt bits.
 *
 *  Using separate core0 and core1 values to do 16-bit reads/writes
 *  because we do not want to overwrite the other cores value.
 */
#define INTERRUPT_CORE_0       (0x40001000)
#define INTERRUPT_CORE_1       (0x40001000 + 2)

static Hwi_FuncPtr userFxn = NULL;

static UInt16 sysm3ProcId;
static UInt16 appm3ProcId;
static UInt16 hostProcId;
static UInt16 dspProcId;


static struct FxnTable {
	Fxn    func;
	UArg   arg;
} table[2];

static UInt32 numPlugged = 0;

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

Void InterruptProxy_intRegister(UInt16 remoteProcId, Hwi_FuncPtr fxn, UArg arg)
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
 *  ======== InterruptDucati_intEnable ========
 *  Enable remote processor interrupt
 */
Void InterruptDucati_intEnable(UInt16 remoteProcId)
{
    /*
     *  If the remote processor communicates via mailboxes, we should enable
     *  the Mailbox IRQ instead of enabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
	if (remoteProcId == MultiProc_getId("HOST")) {
		REG32(MAILBOX_IRQENABLE_SET_M3) = 0x100;
	}
	else if (remoteProcId == MultiProc_getId("DSP")) {
		REG32(MAILBOX_IRQENABLE_SET_M3) = 0x4;
	}
}

/*!
 *  ======== InterruptDucati_intDisable ========
 *  Disables remote processor interrupt
 */
Void InterruptDucati_intDisable(UInt16 remoteProcId)
{
    /*
     *  If the remote processor communicates via mailboxes, we should disable
     *  the Mailbox IRQ instead of disabling the Hwi because multiple mailboxes
     *  share the same Hwi
     */
	if (remoteProcId == MultiProc_getId("HOST")) {
		REG32(MAILBOX_IRQENABLE_CLR_M3) = 0x100;
	}
	else if (remoteProcId == MultiProc_getId("DSP")) {
		REG32(MAILBOX_IRQENABLE_CLR_M3) = 0x4;
	}
}

/*!
 *  ======== InterruptDucati_intRegister ========
 */
Void InterruptDucati_intRegister(UInt16 remoteProcId,
                                 Fxn func, UArg arg)
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

    InterruptDucati_intClear(remoteProcId);

    Hwi_Params_init(&hwiAttrs);
    hwiAttrs.maskSetting = Hwi_MaskingOption_LOWER;

	numPlugged++;
	if (numPlugged == 1) {
		Hwi_create(M3INT_MBX,
				   (Hwi_FuncPtr) InterruptDucati_intShmMbxStub,
				   &hwiAttrs,
				   NULL);

		/* Interrupt_intEnable won't enable the Hwi */
		Hwi_enableInterrupt(M3INT_MBX);
	}

    /* Enable the mailbox interrupt to the M3 core */
    InterruptDucati_intEnable(remoteProcId);

    /* Restore global interrupts */
    Hwi_restore(key);

}

/*!
 *  ======== InterruptDucati_intUnregister ========
 */
Void InterruptDucati_intUnregister(UInt16 remoteProcId)
{
    Hwi_Handle hwiHandle;
    Int index;
    InterruptDucati_FxnTable *table;

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
    InterruptDucati_intDisable(remoteProcId);

    /* Delete/disable the Hwi */

	numPlugged--;
	if (numPlugged == 0) {
		hwiHandle = Hwi_getHandle(M3INT_MBX);
		Hwi_delete(&hwiHandle);
	}

    /* Clear the FxnTable entry for the remote processor */
    table[index].func = NULL;
    table[index].arg  = 0;
}

/*!
 *  ======== InterruptDucati_intSend ========
 *  Send interrupt to the remote processor
 */
Void InterruptDucati_intSend(UInt16 remoteProcId, UArg arg)
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
        if (REG32(MAILBOX_STATUS(M3_TO_DSP)) == 0) {
            REG32(MAILBOX_MESSAGE(M3_TO_DSP)) = arg;
        }
        Hwi_restore(key);
    } else if(remoteProcId == MultiProc_getId("HOST")) {
        key = Hwi_disable();
        if (REG32(MAILBOX_STATUS(M3_TO_HOST)) == 0) {
            REG32(MAILBOX_MESSAGE(M3_TO_HOST)) = arg;
        }
        Hwi_restore(key);
    } else {
        Error_raise(NULL, Error_E_generic, "Invalid remote processor: %d", remoteProcId);
	}
}

/*!
 *  ======== InterruptDucati_intClear ========
 *  Clear interrupt
 */
UInt InterruptDucati_intClear(UInt16 remoteProcId)
{
    UInt arg;

    if(remoteProcId == MultiProc_getId("HOST")) {
		arg = REG32(MAILBOX_MESSAGE(HOST_TO_M3));
		REG32(MAILBOX_IRQSTATUS_CLR_M3) = 0x100; /* Mbx 4 */
	} else if (remoteProcId == MultiProc_getId("DSP")) {
		arg = REG32(MAILBOX_MESSAGE(DSP_TO_M3));
		REG32(MAILBOX_IRQSTATUS_CLR_M3) = 0x4; /* Mbx 1 */
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
 *  ======== InterruptDucati_intShmMbxStub ========
 */
Void InterruptDucati_intShmMbxStub(UArg arg)
{
    InterruptDucati_FxnTable *table;

    /* Process messages from the DSP  */
    if ((REG32(MAILBOX_IRQENABLE_SET_M3) & 0x4) &&
        REG32(MAILBOX_STATUS(DSP_TO_M3)) != 0) {
        (table[0].func)(table[0].arg);
    }

    /* Process messages from the HOST  */
    if ((REG32(MAILBOX_IRQENABLE_SET_M3) & 0x100) &&
        REG32(MAILBOX_STATUS(HOST_TO_M3)) != 0) {
        (table[1].func)(table[1].arg);
    }
}

