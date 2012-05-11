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
/** ============================================================================
 *  @file       MessageQCopy.c
 *
 *  @brief      A simple copy-based MessageQ, to work with Linux virtio_rp_msg.
 *
 *  ============================================================================
 */

/* this define must precede inclusion of any xdc header file */
#define Registry_CURDESC ti_ipc_rpmsg_MessageQCopyS2S__Desc
#define MODULE_NAME "ti.ipc.rpmsg.MessageQCopy"

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Main.h>
#include <xdc/runtime/Registry.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/gates/GateSwi.h>

#include <ti/sdo/utils/List.h>
#include <ti/ipc/MultiProc.h>

#include <string.h>

#include "MessageQCopy_s2s.h"
#include "VirtQueue_s2s.h"

/* =============================================================================
 * Structures & Enums
 * =============================================================================
 */

/* Various arbitrary limits: */
#define MAXREMOTEPROCS		   5
#define MAXENDPOINTS		   256
#define MAXMESSAGEBUFFERS      512
#define MSGBUFFERSIZE          512   // Max payload + sizeof(ListElem)
#define MAXHEAPSIZE            (MAXMESSAGEBUFFERS * MSGBUFFERSIZE)
#define HEAPALIGNMENT          8

static UInt8 recv_buffers[MAXHEAPSIZE];

/* Endpoint pool maps an endpoint to message queue. */
typedef struct MessageQCopyS2S_Object* MessageQCopyS2S_EndptPool[MAXENDPOINTS];
#define MessageQCopyS2S_lookupEndpnt(p, e)	(((MessageQCopyS2S_Object**)(p))[(e)])
#define MessageQCopyS2S_assignEndpnt(p, e, obj)	(((MessageQCopyS2S_Object**)(p))[(e)] = (obj))


/* The MessageQCopy Object */
typedef struct MessageQCopyS2S_Object {
    UInt32           queueId;      /* Unique id (procId | queueIndex)       */
    Semaphore_Handle semHandle;    /* I/O Completion                        */
    List_Handle      queue;        /* Queue of pending messages             */
    Bool             unblocked;    /* Use with signal to unblock _receive() */
    MessageQCopyS2S_EndptPool *pool;  /* track associated pool for cleanup     */
} MessageQCopyS2S_Object;


/* This struct is used as private data supplied to the VQ callback */
/* function.  It is also the passed to the SWI fxn through arg0.   */
typedef struct MessageQCopyS2S_callbackData {
	UInt32					vqId;
	MessageQCopyS2S_EndptPool	*pool;
	Swi_Handle				swi;
} MessageQCopyS2S_callbackData;




/* Module_State */
typedef struct MessageQCopyS2S_Module {
    /* Instance gate: */
    GateSwi_Handle gateSwi;

	/* Lookup vqId from procId for sending.  Array indexed by procId. */
	UInt32 primary_VQs[MAXREMOTEPROCS];

	/* References to callbackData indexed by vqId */
	MessageQCopyS2S_callbackData VQ_callbacks[MAXREMOTEPROCS*2];

	/* Global endpoint pool */
	MessageQCopyS2S_EndptPool global_pool;

	/* Lookup endpoint pool from procId.  Array indexed by procId. */
	/*	MessageQCopyS2S_EndptPool pools[MAXREMOTEPROCS]; */

    /* Heap from which to allocate free messages for copying: */
    HeapBuf_Handle              heap;
} MessageQCopyS2S_Module;

/* For use with module.primary_VQs */
#define NO_PRIMARY_VQ ((UInt32)-1)


/* Message Header: Must match mp_msg_hdr in virtio_rp_msg.h on Linux side. */
typedef struct MessageQCopyS2S_MsgHeader {
    Bits32 srcAddr;                 /* source endpoint addr               */
    Bits32 dstAddr;                 /* destination endpoint addr          */
    Bits32 reserved;                /* reserved                           */
    Bits16 dataLen;                 /* data length                        */
    Bits16 flags;                   /* bitmask of different flags         */
    UInt8  payload[];               /* Data payload                       */
} MessageQCopyS2S_MsgHeader;

typedef MessageQCopyS2S_MsgHeader *MessageQCopyS2S_Msg;

/* Element to hold payload copied onto receiver's queue.                  */
typedef struct Queue_elem {
    List_Elem    elem;              /* Allow list linking.                */
    UInt         len;               /* Length of data                     */
    UInt32       src;               /* Src address/endpt of the msg       */
    Char         data[];            /* payload begins here                */
} Queue_elem;




/* module diags mask */
Registry_Desc Registry_CURDESC;

static MessageQCopyS2S_Module      module;


/* Module ref count: */
static Int curInit = 0;


/*
 *  ======== MessageQCopyS2S_enqueMsg ========
 */
#define FXNN "MessageQCopyS2S_enqueMsg"
Int MessageQCopyS2S_enqueMsg(MessageQCopyS2S_EndptPool *pool, MessageQCopyS2S_Msg msg)
{
    Int               status = MessageQCopyS2S_S_SUCCESS;
	IArg              key;
    MessageQCopyS2S_Handle msgq;
    UInt              size;
    Queue_elem        *payload;

	/* Protect from MessageQCopyS2S_delete */
	key = GateSwi_enter(module.gateSwi);
	msgq = MessageQCopyS2S_lookupEndpnt(pool, msg->dstAddr);
	GateSwi_leave(module.gateSwi, key);

	if (msgq == NULL) {
		Log_print1(Diags_STATUS, FXNN": no object for endpoint: %d",
			   (IArg)(msg->dstAddr));
		status = MessageQCopyS2S_E_NOENDPT;
		return status;
	}

	/* Allocate a buffer to copy the payload: */
	size = msg->dataLen + sizeof(Queue_elem);

	/* HeapBuf_alloc() is non-blocking, so needs protection: */
	key = GateSwi_enter(module.gateSwi);
	payload = (Queue_elem *)HeapBuf_alloc(module.heap, size, 0, NULL);
	GateSwi_leave(module.gateSwi, key);

	if (payload != NULL)  {
		memcpy(payload->data, msg->payload, msg->dataLen);
		payload->len = msg->dataLen;
		payload->src = msg->srcAddr;

		/* Put on the endpoint's queue and signal: */
		List_put(msgq->queue, (List_Elem *)payload);
		Semaphore_post(msgq->semHandle);
	}
	else {
		status = MessageQCopyS2S_E_MEMORY;
		Log_print0(Diags_STATUS, FXNN": HeapBuf_alloc failed!");
	}

	return status;
}
#undef FXNN


/*
 *  ======== MessageQCopyS2S_swiFxn ========
 */
#define FXNN "MessageQCopyS2S_swiFxn"
static Void MessageQCopyS2S_swiFxn(UArg arg0, UArg arg1)
{
	MessageQCopyS2S_callbackData *cbdata = (MessageQCopyS2S_callbackData*)arg0;
    MessageQCopyS2S_Msg  msg;
    VirtQueueS2S_Handle vq;
    Bool              usedBufAdded = FALSE;

    Log_print0(Diags_ENTRY, "--> "FXNN);

	vq = VirtQueueS2S_getHandle(cbdata->vqId);

	/* Process all available buffers: */
    while ((msg = VirtQueueS2S_getUsedBuf(vq)) != NULL) {

        Log_print3(Diags_USER1, FXNN": \n\tReceived Simm msg: from: 0x%x, "
                   "to: 0x%x, dataLen: %d",
                  (IArg)msg->srcAddr, (IArg)msg->dstAddr, (IArg)msg->dataLen);

        /* Pass to desitination queue (which is on this proc): */
        MessageQCopyS2S_enqueMsg(cbdata->pool, msg);

        VirtQueueS2S_addAvailBuf(vq, msg);
        usedBufAdded = FALSE;
    }

    if (usedBufAdded)  {
       /* Tell host we've processed the buffers: */
       VirtQueueS2S_kick(vq);
    }

    Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN


#define FXNN "callback_availBufReady"
static Void callback_availBufReady(UArg priv)
{
	MessageQCopyS2S_callbackData *cbdata = (MessageQCopyS2S_callbackData*)priv;

    if (cbdata->swi)  {
       /* Post a SWI to process all incoming messages */
        Log_print1(Diags_USER1, FXNN": virtQueue %d kicked", cbdata->vqId);
        Swi_post(cbdata->swi);
    }
}
#undef FXNN

/* =============================================================================
 *  MessageQCopy Functions:
 * =============================================================================
 */

/*
 *  ======== MessasgeQCopy_init ========
 *
 *
 */
#define FXNN "MessageQCopyS2S_init"
Void MessageQCopyS2S_init()
{
    GateSwi_Params gatePrms;
    HeapBuf_Params prms;
    UInt     p;
    Registry_Result result;
	UInt32 tx_vqId, rx_vqId;
	Swi_Params	params;

    Log_print0(Diags_ENTRY, "--> "FXNN);

    /* register with xdc.runtime to get a diags mask */
    result = Registry_addModule(&Registry_CURDESC, MODULE_NAME);
    Assert_isTrue(result == Registry_SUCCESS, (Assert_Id)NULL);

	Assert_isTrue(MultiProc_getNumProcessors() <= MAXREMOTEPROCS, (Assert_Id)NULL);

	/* Initialize the pool to invalid values. */
	memset(module.global_pool, 0, sizeof(module.global_pool));
	memset(module.primary_VQs, -1, sizeof(module.primary_VQs));
	memset(module.VQ_callbacks, 0, sizeof(module.VQ_callbacks));

    /* Gate to protect module object and lists: */
    GateSwi_Params_init(&gatePrms);
    module.gateSwi = GateSwi_create(&gatePrms, NULL);

    HeapBuf_Params_init(&prms);
    prms.blockSize    = MSGBUFFERSIZE;
    prms.numBlocks    = MAXMESSAGEBUFFERS;
    prms.buf          = recv_buffers;
    prms.bufSize      = MAXHEAPSIZE;
    prms.align        = HEAPALIGNMENT;
    module.heap       = HeapBuf_create(&prms, NULL);
    if (module.heap == NULL) {
       System_abort("MessageQCopyS2S_init: HeapBuf_create failed\n");
    }

	/* Setup the module's key data structures that control the  */
	/* releationship between VQs, msgQs and endpoint pools.     */
	for(p = 0; p < MultiProc_getNumProcessors(); p++) {
		if(VirtioIPC_getVirtQueues(VirtioIPC_RPMSG, p, 1, &tx_vqId, &rx_vqId)) {
			/* setup the receiving path */
			module.VQ_callbacks[rx_vqId].pool = &module.global_pool;
			module.VQ_callbacks[rx_vqId].vqId = rx_vqId;

			Swi_Params_init(&params);
			params.arg0 = (UArg)&module.VQ_callbacks[rx_vqId];
			module.VQ_callbacks[rx_vqId].swi = Swi_create(MessageQCopyS2S_swiFxn, &params, NULL);
			VirtQueueS2S_setCallback(rx_vqId, callback_availBufReady, (UArg)&module.VQ_callbacks[rx_vqId]);

			/* setup the sending path */
			module.primary_VQs[p] = tx_vqId;
			module.VQ_callbacks[tx_vqId].pool = 0;
			module.VQ_callbacks[tx_vqId].vqId = tx_vqId;
			module.VQ_callbacks[tx_vqId].swi = NULL;
		} else {
			module.primary_VQs[p] = NO_PRIMARY_VQ;
		}
	}

    Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN

/*
 *  ======== MessasgeQCopy_finalize ========
 */
#define FXNN "MessageQCopyS2S_finalize"
Void MessageQCopyS2S_finalize()
{
	UInt32 vqId;
	UInt   p;

	Log_print0(Diags_ENTRY, "--> "FXNN);

	for(p = 0; p < MultiProc_getNumProcessors(); p++) {
		vqId = module.primary_VQs[p];
		if(vqId != NO_PRIMARY_VQ) {
			Swi_delete(&(module.VQ_callbacks[vqId].swi));
		}
	}

	/* Tear down Module: */
	HeapBuf_delete(&(module.heap));

	GateSwi_delete(&module.gateSwi);

	Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN


/*
 *  ======== MessageQCopyS2S_create ========
 */
#define FXNN "MessageQCopyS2S_create"
MessageQCopyS2S_Handle MessageQCopyS2S_rawCreate(MessageQCopyS2S_EndptPool* pool, UInt32 reserved, UInt32 * endpoint)
{
    MessageQCopyS2S_Object    *obj = NULL;
    Bool                   found = FALSE;
    Int                    i;
    UInt16                 queueIndex = 0;
    IArg key;

    Log_print2(Diags_ENTRY, "--> "FXNN": (reserved=%d, endpoint=0x%x)",
                (IArg)reserved, (IArg)endpoint);

    Assert_isTrue((curInit > 0) , NULL);

    key = GateSwi_enter(module.gateSwi);

    if (reserved == MessageQCopyS2S_ASSIGN_ANY)  {
       /* Search the array for a free slot above reserved: */
       for (i = MessageQCopyS2S_MAX_RESERVED_ENDPOINT + 1;
                  (i < MAXENDPOINTS) && (found == FALSE) ; i++) {
           if (MessageQCopyS2S_lookupEndpnt(pool, i) == NULL) {
            queueIndex = i;
            found = TRUE;
            break;
           }
       }
    }
    else if ((queueIndex = reserved) <= MessageQCopyS2S_MAX_RESERVED_ENDPOINT) {
       if (MessageQCopyS2S_lookupEndpnt(pool, i) == NULL) {
           found = TRUE;
       }
    }

    if (found)  {
       obj = Memory_alloc(NULL, sizeof(MessageQCopyS2S_Object), 0, NULL);
       if (obj != NULL) {
           /* Allocate a semaphore to signal when messages received: */
           obj->semHandle = Semaphore_create(0, NULL, NULL);

           /* Create our queue of to be received messages: */
           obj->queue = List_create(NULL, NULL);

           /* Store our endpoint, and object: */
           obj->queueId = queueIndex;
           MessageQCopyS2S_assignEndpnt(pool, queueIndex, obj);

           /* See MessageQCopyS2S_unblock() */
           obj->unblocked = FALSE;

           *endpoint    = queueIndex;
           Log_print1(Diags_LIFECYCLE, FXNN": endPt created: %d",
                        (IArg)queueIndex);
       }
    }

    GateSwi_leave(module.gateSwi, key);

    Log_print1(Diags_EXIT, "<-- "FXNN": 0x%x", (IArg)obj);
    return (obj);
}
#undef FXNN


MessageQCopyS2S_Handle MessageQCopyS2S_create(UInt32 reserved, UInt32 * endpoint)
{
    return MessageQCopyS2S_rawCreate(&module.global_pool, reserved, endpoint);
}


/*
 *  ======== MessageQCopyS2S_delete ========
 */
#define FXNN "MessageQCopyS2S_delete"
Int MessageQCopyS2S_delete(MessageQCopyS2S_Handle *handlePtr)
{
    Int                    status = MessageQCopyS2S_S_SUCCESS;
    MessageQCopyS2S_Object    *obj;
    Queue_elem             *payload;
    IArg                   key;

    Log_print1(Diags_ENTRY, "--> "FXNN": (handlePtr=0x%x)", (IArg)handlePtr);

    Assert_isTrue((curInit > 0) , NULL);

    if (handlePtr && (obj = (MessageQCopyS2S_Object *)(*handlePtr)))  {

       /* Null out our slot in the endpoint pool. */
       key = GateSwi_enter(module.gateSwi);
       MessageQCopyS2S_assignEndpnt(obj->pool, obj->queueId, NULL);
       obj->pool = NULL;
       GateSwi_leave(module.gateSwi, key);

       Semaphore_delete(&(obj->semHandle));

       /* Free/discard all queued message buffers: */
       while ((payload = (Queue_elem *)List_get(obj->queue)) != NULL) {
           HeapBuf_free(module.heap, (Ptr)payload, MSGBUFFERSIZE);
       }

       List_delete(&(obj->queue));

       Log_print1(Diags_LIFECYCLE, FXNN": endPt deleted: %d",
                        (IArg)obj->queueId);

       /* Now free the obj */
       Memory_free(NULL, obj, sizeof(MessageQCopyS2S_Object));

       *handlePtr = NULL;
    }

    Log_print1(Diags_EXIT, "<-- "FXNN": %d", (IArg)status);
    return(status);
}
#undef FXNN

/*
 *  ======== MessageQCopyS2S_recv ========
 */
#define FXNN "MessageQCopyS2S_recv"
Int MessageQCopyS2S_recv(MessageQCopyS2S_Handle handle, Ptr data, UInt16 *len,
                      UInt32 *rplyEndpt, UInt timeout)
{
    Int                 status = MessageQCopyS2S_S_SUCCESS;
    MessageQCopyS2S_Object *obj = (MessageQCopyS2S_Object *)handle;
    Bool                semStatus;
    Queue_elem          *payload;

    Log_print5(Diags_ENTRY, "--> "FXNN": (handle=0x%x, data=0x%x, len=0x%x,"
               "rplyEndpt=0x%x, timeout=%d)", (IArg)handle, (IArg)data,
               (IArg)len, (IArg)rplyEndpt, (IArg)timeout);

    Assert_isTrue((obj->pool != NULL), NULL);

    /* Check vring for pending messages before we block: */
//    Swi_post(transport.swiHandle);  /* cant check, no access to any swi */

    /*  Block until notified. */
    semStatus = Semaphore_pend(obj->semHandle, timeout);

    if (semStatus == FALSE)  {
       status = MessageQCopyS2S_E_TIMEOUT;
       Log_print0(Diags_STATUS, FXNN": Sem pend timeout!");
    }
    else if (obj->unblocked) {
       status = MessageQCopyS2S_E_UNBLOCKED;
    }
    else  {
       payload = (Queue_elem *)List_get(obj->queue);

       if (!payload) {
           System_abort("MessageQCopyS2S_recv: got a NULL payload\n");
       }
    }

    if (status == MessageQCopyS2S_S_SUCCESS)  {
       /* Now, copy payload to client and free our internal msg */
       memcpy(data, payload->data, payload->len);
       *len = payload->len;
       *rplyEndpt = payload->src;

       HeapBuf_free(module.heap, (Ptr)payload,
                    (payload->len + sizeof(Queue_elem)));
    }

    Log_print1(Diags_EXIT, "<-- "FXNN": %d", (IArg)status);
    return (status);
}
#undef FXNN


/*
 *  ======== MessageQCopyS2S_rawSend ========
 */
#define FXNN "MessageQCopyS2S_rawSend"
Int MessageQCopyS2S_rawSend(UInt32 vqId,
                      UInt32 dstEndpt,
                      UInt32 srcEndpt,
                      Ptr    data,
                      UInt16 len)
{
    Int               status = MessageQCopyS2S_S_SUCCESS;
    Int16             token = 0;
    VirtQueueS2S_Handle vq;
    MessageQCopyS2S_Msg msg;
    IArg              key;
    Int              length;
	UInt              bufSize;

    Log_print5(Diags_ENTRY, "--> "FXNN": (vqId=%d, dstEndpt=%d, "
               "srcEndpt=%d, data=0x%x, len=%d", (IArg)vqId, (IArg)dstEndpt,
               (IArg)srcEndpt, (IArg)data, (IArg)len);

    Assert_isTrue((curInit > 0) , NULL);

	vq = VirtQueueS2S_getHandle(vqId);
    bufSize = sizeof(msg) + len;

	/* Send to remote processor: */
	key = GateSwi_enter(module.gateSwi);  // Protect vring structs.
	token = VirtQueueS2S_getAvailBuf(vq, (Void **)&msg, &length);
	GateSwi_leave(module.gateSwi, key);

	if(length < bufSize) {
		status = MessageQCopyS2S_E_FAIL;
		Log_print1(Diags_STATUS, FXNN": buffer from vq %d to small for data", vqId);
		return status;
	}
	if (token >= 0) {
		/* Copy the payload and set message header: */
		memcpy(msg->payload, data, len);
		msg->dataLen = len;
		msg->dstAddr = dstEndpt;
		msg->srcAddr = srcEndpt;
		msg->flags = 0;
		msg->reserved = 0;

		key = GateSwi_enter(module.gateSwi);  // Protect vring structs.
		VirtQueueS2S_addUsedBuf(vq, token, bufSize);
		VirtQueueS2S_kick(vq);
		GateSwi_leave(module.gateSwi, key);
	} else {
		status = MessageQCopyS2S_E_FAIL;
		Log_print0(Diags_STATUS, FXNN": getAvailBuf failed!");
	}

    Log_print1(Diags_EXIT, "<-- "FXNN": %d", (IArg)status);
    return (status);
}
#undef FXNN


/*
 *  ======== MessageQCopyS2S_send ========
 */
Int MessageQCopyS2S_send(UInt16 procid, UInt32 dstEndpt, UInt32 srcEndpt,
                      Ptr data, UInt16 len)
{
	UInt32 vqId;

	if(procid >= MAXREMOTEPROCS)
		return MessageQCopyS2S_E_FAIL;

	if(procid == MultiProc_self())
		return MessageQCopyS2S_E_FAIL;

	vqId = module.primary_VQs[procid];
	return MessageQCopyS2S_rawSend(vqId, dstEndpt, srcEndpt, data, len);
}


/*
 *  ======== MessageQCopyS2S_unblock ========
 */
#define FXNN "MessageQCopyS2S_unblock"
Void MessageQCopyS2S_unblock(MessageQCopyS2S_Handle handle)
{
    MessageQCopyS2S_Object *obj = (MessageQCopyS2S_Object *)handle;

    Log_print1(Diags_ENTRY, "--> "FXNN": (handle=0x%x)", (IArg)handle);

    /* Set instance to 'unblocked' state, and post */
    obj->unblocked = TRUE;
    Semaphore_post(obj->semHandle);
    Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN
