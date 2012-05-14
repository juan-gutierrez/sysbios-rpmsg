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
*  ======== m3_dsp_task.c ========
 *
 *  Works with the rpmsg_client_sample and rpmsg_server_sample Linux drivers.
 */

#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>

#include <ti/ipc/MultiProc.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <ti/ipc/rpmsg/VirtQueue_s2s.h>
#include <ti/ipc/rpmsg/MessageQCopy_s2s.h>
#include <ti/srvmgr/NameMap.h>
//#include <ti/resmgr/IpcResource.h>

#define APP_NUM_ITERATIONS 100000

Void messageQcopyS2S_taskFxn(UArg arg0, UArg arg1)
{
	MessageQCopyS2S_Handle    handle;
	Char                   buffer[128];
	UInt32                 myEndpoint = 0;
	UInt32                 remoteEndpoint;
	UInt16                 dstProc;
	UInt16                 len;
	Int                    i;
	Int status = 0;

	System_printf("MessageQCopyS2S_taskFxn Entered, dstProc 0x%x\n", arg0);

	dstProc = arg0;

	VirtioIPC_init((void*)0xa0100000);
	MessageQCopyS2S_init();

{
    volatile int x=1;
    while(x);
}

	/* Create the messageQ for receiving (and get our endpoint for sending). */
	if (MultiProc_self() > dstProc) {
		handle = MessageQCopyS2S_create(222, &myEndpoint);
		len = snprintf(buffer, sizeof(buffer), "HELLO_FROM_%s", MultiProc_getName(MultiProc_self()));
		/* Task_sleep(3000); */
		System_printf("Sending msg %s from %s to %s...\n", buffer, MultiProc_getName(MultiProc_self()),
					MultiProc_getName(dstProc));
		status = MessageQCopyS2S_send(dstProc, 111, myEndpoint, &buffer, len);
		if (status) {
			System_printf("MessageQCopyS2S_taskFxn: MessageQCopyS2S_send "
						" failed status %d\n", status);
		}
	} else {
		handle = MessageQCopyS2S_create(111, &myEndpoint);
	}
	for (i = 0; i < 25; i++) {
		/* Await a character message: */
		System_printf("MessageQCopyS2S_taskFxn: waiting for msg\n");
		MessageQCopyS2S_recv(handle, (Ptr)buffer, &len, &remoteEndpoint,
						 MessageQCopyS2S_FOREVER);

		buffer[len] = '\0';
		System_printf("MessageQCopyS2S_taskFxn Received len:%d from %d data: %s\n",
					len, remoteEndpoint, buffer);

		/* Send data back to remote endpoint: */
		len = snprintf(buffer, sizeof(buffer), "HELLO_BACK_FROM_%s_%d", MultiProc_getName(MultiProc_self()), i);
		System_printf("Sending msg %s from %s to %s...\n", buffer, MultiProc_getName(MultiProc_self()),
					MultiProc_getName(dstProc));
		status = MessageQCopyS2S_send(dstProc, remoteEndpoint, myEndpoint, (Ptr)buffer, len);
		if (status) {
			System_printf("MessageQCopyS2S_taskFxn: MessageQCopyS2S_send "
						" failed status %d\n", status);
		}
	}
	System_printf("MessageQCopyS2S_taskFxn: Finished\n");

}

Void start_MessageQCopyS2S_task(Void)
{
    Task_Params params;
    UInt32 dstproc = (UInt32)-1;

#ifdef CORE0
    dstproc = MultiProc_getId("DSP");
#endif
#ifdef DSP
    dstproc = MultiProc_getId("CORE0");
#endif

    /* Respond to ping tests from Linux side rpmsg sample drivers: */
    Task_Params_init(&params);
    params.instance->name = "MessageQcopyS2S_test";
    params.priority = 3;
    params.arg0 = dstproc;
    Task_create(messageQcopyS2S_taskFxn, &params, NULL);
}
