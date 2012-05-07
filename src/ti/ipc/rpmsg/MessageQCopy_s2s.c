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
#define Registry_CURDESC ti_ipc_rpmsg_MessageQCopy__Desc
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

#include "MessageQCopy.h"
#include "VirtQueue.h"

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

/* The MessageQCopy Object */
typedef struct MessageQCopy_Object {
    UInt32           queueId;      /* Unique id (procId | queueIndex)       */
    Semaphore_Handle semHandle;    /* I/O Completion                        */
    List_Handle      queue;        /* Queue of pending messages             */
    Bool             unblocked;    /* Use with signal to unblock _receive() */
    MessageQCopy_EndptPool *pool;  /* track associated pool for cleanup     */
} MessageQCopy_Object;


/* This struct is used as private data supplied to the VQ callback */
/* function.  It is also the passed to the SWI through arg0.       */
typdef struct MessageQCopy_callbackData {
	UInt32				vqid;
	VirtQueue_Handle	*pool;
	Swi_Handle			swi;
} MessageQCopy_callbackData;


/* Endpoint pool maps an endpoint to message queue. */
typedef MessageQCopy_Object*[MAXENDPOINTS] MessageQCopy_EndptPool;
#define MessageQCopy_lookupEndpnt(p, e)	(((MessageQCopy_Object*)(p))[(e)])
#define MessageQCopy_assignEndpnt(p, e, obj)	(((MessageQCopy_Object*)(p))[(e)] = (obj))


/* Module_State */
typedef struct MessageQCopy_Module {
    /* Instance gate: */
    GateSwi_Handle gateSwi;

	/* Lookup vqId from procId for sending.  Array indexed by procId. */
	UInt32 primary_VQs[MAXREMOTEPROCS];

	/* References to callbackData indexed by vqid */
	MessageQCopy_callbackData VQ_callbacks[MAXREMOTEPROCS*2];

	/* Global endpoint pool */
	MessageQCopy_EndptPool global_pool;

	/* Lookup endpoint pool from procId.  Array indexed by procId. */
	/*	MessageQCopy_EndptPool pools[MAXREMOTEPROCS]; */

    /* Heap from which to allocate free messages for copying: */
    HeapBuf_Handle              heap;
} MessageQCopy_Module;

/* For use with module.primary_VQs */
#define NO_PRIMARY_VQ ((UInt32)-1);


/* Message Header: Must match mp_msg_hdr in virtio_rp_msg.h on Linux side. */
typedef struct MessageQCopy_MsgHeader {
    Bits32 srcAddr;                 /* source endpoint addr               */
    Bits32 dstAddr;                 /* destination endpoint addr          */
    Bits32 reserved;                /* reserved                           */
    Bits16 dataLen;                 /* data length                        */
    Bits16 flags;                   /* bitmask of different flags         */
    UInt8  payload[];               /* Data payload                       */
} MessageQCopy_MsgHeader;

typedef MessageQCopy_MsgHeader *MessageQCopy_Msg;

/* Element to hold payload copied onto receiver's queue.                  */
typedef struct Queue_elem {
    List_Elem    elem;              /* Allow list linking.                */
    UInt         len;               /* Length of data                     */
    UInt32       src;               /* Src address/endpt of the msg       */
    Char         data[];            /* payload begins here                */
} Queue_elem;




/* module diags mask */
Registry_Desc Registry_CURDESC;

static MessageQCopy_Module      module;


/* Module ref count: */
static Int curInit = 0;

/*
 *  ======== MessageQCopy_swiFxn ========
 */
#define FXNN "MessageQCopy_swiFxn"
static Void MessageQCopy_swiFxn(UArg arg0, UArg arg1)
{
	MessageQCopy_swiData *swiData = (MessageQCopy_swiData *)arg0;
    MessageQCopy_Msg  msg;
    VirtQueue_Handle vq;
    Bool              usedBufAdded = FALSE;

    Log_print0(Diags_ENTRY, "--> "FXNN);

	vq = VirtQueue_get(swiData->vqid);

	/* Process all available buffers: */
    while ((msg = VirtQueue_getUsedBuf(vq)) != NULL) {

        Log_print3(Diags_USER1, FXNN": \n\tReceived Simm msg: from: 0x%x, "
                   "to: 0x%x, dataLen: %d",
                  (IArg)msg->srcAddr, (IArg)msg->dstAddr, (IArg)msg->dataLen);

        /* Pass to desitination queue (which is on this proc): */
        MessageQCopy_enqueMsg(swoData->pool, msg);

        VirtQueue_addAvailBuf(vq, msg);
        usedBufAdded = FALSE;
    }

    if (usedBufAdded)  {
       /* Tell host we've processed the buffers: */
       VirtQueue_kick(vq);
    }

    Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN


#define FXNN "callback_availBufReady"
static Void callback_availBufReady(Ptr priv)
{
	MessageQCopy_callbackData *cbd = (MessageQCopy_callbackData*)priv;

    if (cbd->swi)  {
       /* Post a SWI to process all incoming messages */
        Log_print1(Diags_USER1, FXNN": virtQueue %d kicked", cdb->vqid);
        Swi_post(cbd->swi);
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
#define FXNN "MessageQCopy_init"
Void MessageQCopy_init(UInt16 remoteProcId)
{
    GateSwi_Params gatePrms;
    HeapBuf_Params prms;
    int     i;
    Registry_Result result;
	UInt32 tx_vqid, rx_vqid;
	Swi_Params	params;

    Log_print1(Diags_ENTRY, "--> "FXNN": (remoteProcId=%d)",
                (IArg)remoteProcId);

    /* register with xdc.runtime to get a diags mask */
    result = Registry_addModule(&Registry_CURDESC, MODULE_NAME);
    Assert_isTrue(result == Registry_SUCCESS, (Assert_Id)NULL);

	Assert_isTrue(MultiProc_getNumProcessors() <= MAXREMOTEPROCS);

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
    if (module.heap == 0) {
       System_abort("MessageQCopy_init: HeapBuf_create returned 0\n");
    }

	/* Setup the module's key data structures that control the  */
	/* releationship between VQs, msgQs and endpoint pools.     */
	for(p = 0; p < MultiProc_getNumProcessors(); p++) {
		if(VirtioIPC_getVirtQueue(VirtioIPC_RPMSG, p, 1, &tx_vqid, &rx_vqid)) {
			/* setup the receiving path */
			module.VQ_callbacks[rx_vqid].pool = &module.global_pool;
			module.VQ_callbacks[rx_vqid].vqid = rx_vqid;

			Swi_Params_init(&params);
			params.arg0 = &module.VQ_callbacks[rx_vqid];
			module.VQ_callbacks[rx_vqid].swi = Swi_create(MessageQCopy_swiFxn, &params, NULL);
			VirtQueue_setCallback(rx_vqid, callback_availBufReady, (Void*)&module.VQ_callbacks[rx_vqid]);

			/* setup the sending path */
			module.primary_VQs[p] = tx_vqid;
			module.VQ_callbacks[tx_vqid].pool = 0;
			module.VQ_callbacks[tx_vqid].vqid = tx_vqid;
			module.VQ_callbacks[tx_vqid].swi = NULL;
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
#define FXNN "MessageQCopy_finalize"
Void MessageQCopy_finalize()
{
	UInt32 vqid;
	Log_print0(Diags_ENTRY, "--> "FXNN);

	for(p = 0; p < MultiProc_getNumProcessors(); p++) {
		if((vqid = module.primary_VQs[p]) != NO_PRIMARY_VQ) {
			Swi_delete(&(module.VQ_callbacks[vqid].swi));
		}
	}

	/* Tear down Module: */
	HeapBuf_delete(&(module.heap));

	GateSwi_delete(&module.gateSwi);

	Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN




MessageQCopy_Handle MessageQCopy_create(UInt32 reserved, UInt32 * endpoint)
{
    return MessageQCopy_rawCreate(&module.global_pool, reserved, endpoint);
}


/*
 *  ======== MessageQCopy_create ========
 */
#define FXNN "MessageQCopy_create"
MessageQCopy_Handle MessageQCopy_rawCreate(MessageQCopy_EndptPool* pool, UInt32 reserved, UInt32 * endpoint)
{
    MessageQCopy_Object    *obj = NULL;
    Bool                   found = FALSE;
    Int                    i;
    UInt16                 queueIndex = 0;
    IArg key;

    Log_print2(Diags_ENTRY, "--> "FXNN": (reserved=%d, endpoint=0x%x)",
                (IArg)reserved, (IArg)endpoint);

    Assert_isTrue((curInit > 0) , NULL);

    key = GateSwi_enter(module.gateSwi);

    if (reserved == MessageQCopy_ASSIGN_ANY)  {
       /* Search the array for a free slot above reserved: */
       for (i = MessageQCopy_MAX_RESERVED_ENDPOINT + 1;
                  (i < MAXENDPOINTS) && (found == FALSE) ; i++) {
           if (MessageQCopy_lookupEndpnt(pool, i) == NULL) {
            queueIndex = i;
            found = TRUE;
            break;
           }
       }
    }
    else if ((queueIndex = reserved) <= MessageQCopy_MAX_RESERVED_ENDPOINT) {
       if (MessageQCopy_lookupEndpnt(pool, i) == NULL) {
           found = TRUE;
       }
    }

    if (found)  {
       obj = Memory_alloc(NULL, sizeof(MessageQCopy_Object), 0, NULL);
       if (obj != NULL) {
           /* Allocate a semaphore to signal when messages received: */
           obj->semHandle = Semaphore_create(0, NULL, NULL);

           /* Create our queue of to be received messages: */
           obj->queue = List_create(NULL, NULL);

           /* Store our endpoint, and object: */
           obj->queueId = queueIndex;
           MessageQCopy_assignEndpnt(pool, queueIndex, obj);

           /* See MessageQCopy_unblock() */
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

/*
 *  ======== MessageQCopy_delete ========
 */
#define FXNN "MessageQCopy_delete"
Int MessageQCopy_delete(MessageQCopy_Handle *handlePtr)
{
    Int                    status = MessageQCopy_S_SUCCESS;
    MessageQCopy_Object    *obj;
    Queue_elem             *payload;
    IArg                   key;

    Log_print1(Diags_ENTRY, "--> "FXNN": (handlePtr=0x%x)", (IArg)handlePtr);

    Assert_isTrue((curInit > 0) , NULL);

    if (handlePtr && (obj = (MessageQCopy_Object *)(*handlePtr)))  {

       /* Null out our slot in the endpoint pool. */
       key = GateSwi_enter(module.gateSwi);
       MessageQCopy_assignEndpnt(obj->pool, obj->queueId, NULL);
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
       Memory_free(NULL, obj, sizeof(MessageQCopy_Object));

       *handlePtr = NULL;
    }

    Log_print1(Diags_EXIT, "<-- "FXNN": %d", (IArg)status);
    return(status);
}
#undef FXNN

/*
 *  ======== MessageQCopy_recv ========
 */
#define FXNN "MessageQCopy_recv"
Int MessageQCopy_recv(MessageQCopy_Handle handle, Ptr data, UInt16 *len,
                      UInt32 *rplyEndpt, UInt timeout)
{
    Int                 status = MessageQCopy_S_SUCCESS;
    MessageQCopy_Object *obj = (MessageQCopy_Object *)handle;
    Bool                semStatus;
    Queue_elem          *payload;

    Log_print5(Diags_ENTRY, "--> "FXNN": (handle=0x%x, data=0x%x, len=0x%x,"
               "rplyEndpt=0x%x, timeout=%d)", (IArg)handle, (IArg)data,
               (IArg)len, (IArg)rplyEndpt, (IArg)timeout);

    Assert_isTrue(obj->pool != NULL) , NULL);

    /* Check vring for pending messages before we block: */
//    Swi_post(transport.swiHandle);  /* cant check, no access to any swi */

    /*  Block until notified. */
    semStatus = Semaphore_pend(obj->semHandle, timeout);

    if (semStatus == FALSE)  {
       status = MessageQCopy_E_TIMEOUT;
       Log_print0(Diags_STATUS, FXNN": Sem pend timeout!");
    }
    else if (obj->unblocked) {
       status = MessageQCopy_E_UNBLOCKED;
    }
    else  {
       payload = (Queue_elem *)List_get(obj->queue);

       if (!payload) {
           System_abort("MessageQCopy_recv: got a NULL payload\n");
       }
    }

    if (status == MessageQCopy_S_SUCCESS)  {
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
 *  ======== MessageQCopy_send ========
 */
Int MessageQCopy_send(UInt16 procid, UInt32 dstEndpt, UInt32 srcEndpt,
                      Ptr data, UInt16 len)
{
	UInt32 vqid;

	if(procid >= MAXREMOTEPROCS)
		return MessageQCopy_E_FAIL;

	vqid = module.primaryVQs[procid];
	return MessageQCopy_rawSend(vqid, dstEndpt, srcEndpt, data, len);
}


/*
 *  ======== MessageQCopy_rawSend ========
 */
#define FXNN "MessageQCopy_rawSend"
Int MessageQCopy_rawSend(UInt32 vqid,
                      UInt32 dstEndpt,
                      UInt32 srcEndpt,
                      Ptr    data,
                      UInt16 len)
{
    Int               status = MessageQCopy_S_SUCCESS;
    MessageQCopy_Object   *obj;
    Int16             token = 0;
    VirtQueue_Handle vq;
    MessageQCopy_Msg  msg;
    IArg              key;
    int length;

    Log_print5(Diags_ENTRY, "--> "FXNN": (dstProc=%d, dstEndpt=%d, "
               "srcEndpt=%d, data=0x%x, len=%d", (IArg)dstProc, (IArg)dstEndpt,
               (IArg)srcEndpt, (IArg)data, (IArg)len);

    Assert_isTrue((curInit > 0) , NULL);

    if (dstProc != MultiProc_self()) {
        /* Send to remote processor: */
        key = GateSwi_enter(module.gateSwi);  // Protect vring structs.
		vq = VirtQueue_get(swiData->vqid);
        token = VirtQueue_getAvailBuf(vq,
                                      (Void **)&msg);
        GateSwi_leave(module.gateSwi, key);

        if (token >= 0) {
            /* Copy the payload and set message header: */
            memcpy(msg->payload, data, len);
            msg->dataLen = len;
            msg->dstAddr = dstEndpt;
            msg->srcAddr = srcEndpt;
            msg->flags = 0;
            msg->reserved = 0;

            key = GateSwi_enter(module.gateSwi);  // Protect vring structs.
            VirtQueue_addUsedBuf(vq, token);
            VirtQueue_kick(vq);
            GateSwi_leave(module.gateSwi, key);
		} else {
            status = MessageQCopy_E_FAIL;
            Log_print0(Diags_STATUS, FXNN": getAvailBuf failed!");
        }
    } else {
		/* try to enque msg */
			return MessageQCopy_E_FAIL;
	}

    Log_print1(Diags_EXIT, "<-- "FXNN": %d", (IArg)status);
    return (status);
}
#undef FXNN


/*
 *  ======== MessageQCopy_enqueMsg ========
 */
#define FXNN "MessageQCopy_send"
Int MessageQCopy_enqueMsg(MessageQCopy_EndptPool *pool, MessageQCopy_Msg msg)
{
    Int               status = MessageQCopy_S_SUCCESS;
	IArg              key;
    MessageQCopy_Object msgq;
    UInt              size;
    Queue_elem        *payload;

	/* Protect from MessageQCopy_delete */
	key = GateSwi_enter(module.gateSwi);
	msgq = MessageQCopy_lookupEndpnt(pool, msg->dstAddr);
	GateSwi_leave(module.gateSwi, key);

	if (msgq == NULL) {
		Log_print1(Diags_STATUS, FXNN": no object for endpoint: %d",
			   (IArg)(msg->dstEndpt));
		status = MessageQCopy_E_NOENDPT;
		return status;
	}

	/* Allocate a buffer to copy the payload: */
	size = msg->len + sizeof(Queue_elem);

	/* HeapBuf_alloc() is non-blocking, so needs protection: */
	key = GateSwi_enter(module.gateSwi);
	payload = (Queue_elem *)HeapBuf_alloc(module.heap, size, 0, NULL);
	GateSwi_leave(module.gateSwi, key);

	if (payload != NULL)  {
		memcpy(payload->data, data, len);
		payload->len = msg->len;
		payload->src = msg->srcEndpt;

		/* Put on the endpoint's queue and signal: */
		List_put(msgq->queue, (List_Elem *)payload);
		Semaphore_post(msgq->semHandle);
	}
	else {
		status = MessageQCopy_E_MEMORY;
		Log_print0(Diags_STATUS, FXNN": HeapBuf_alloc failed!");
	}

	return status;
}


/*
 *  ======== MessageQCopy_unblock ========
 */
#define FXNN "MessageQCopy_unblock"
Void MessageQCopy_unblock(MessageQCopy_Handle handle)
{
    MessageQCopy_Object *obj = (MessageQCopy_Object *)handle;

    Log_print1(Diags_ENTRY, "--> "FXNN": (handle=0x%x)", (IArg)handle);

    /* Set instance to 'unblocked' state, and post */
    obj->unblocked = TRUE;
    Semaphore_post(obj->semHandle);
    Log_print0(Diags_EXIT, "<-- "FXNN);
}
#undef FXNN
