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
 *  @file       VirtQueue.c
 *
 *  @brief      Virtio Queue implementation for BIOS
 *
 *  Differences between BIOS version and Linux kernel (include/linux/virtio.h):
 *  - Renamed module from virtio.h to VirtQueueS2S_Object.h to match the API prefixes;
 *  - BIOS (XDC) types and CamelCasing used;
 *  - virtio_device concept removed (i.e, assumes no containing device);
 *  - simplified scatterlist from Linux version;
 *  - VirtQueueS2S_Objects are created statically here, so just added a VirtQueueS2S_Object_init()
 *    fxn to take the place of the Virtio vring_new_virtqueue() API;
 *  - The notify function is implicit in the implementation, and not provided
 *    by the client, as it is in Linux virtio.
 *
 *  All VirtQueue operations can be called in any context.
 *
 *  The virtio header should be included in an application as follows:
 *  @code
 *  #include <ti/ipc/rpmsg/VirtQueue.h>
 *  @endcode
 *
 */

#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Diags.h>

#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Cache.h>

#include <ti/ipc/rpmsg/InterruptProxy.h>
#include <ti/ipc/rpmsg/VirtQueueS2S_s2s.h>
#include <ti/ipc/MultiProc.h>

#include <ti/resources/IpcMemory.h>

#include <string.h>

#include "virtio_ring.h"

/* Used for defining the size of the virtqueue registry */
#define MAX_VIRTQUEUES          10
#define MAX_VQ_PAIRS			((MAX_VIRTQUEUES+1)/2)

/* Predefined device addresses */
#define IPU_MEM_VRING0          0xA0000000
#define IPU_MEM_VRING1          0xA0004000

#define CONSOLE_VRING0_PA       0xA0008000
#define CONSOLE_VRING1_PA       0xA000c000

#define IPU_MEM_VRING2          0xA0010000
#define IPU_MEM_VRING3          0xA0014000

/*
 * Sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of two)
 */
#define VQ0_SIZE                256
#define VQ1_SIZE                256
#define VQ2_SIZE                256
#define VQ3_SIZE                256

/*
 * enum - Predefined Mailbox Messages
 *
 * @RP_MSG_MBOX_READY: informs the M3's that we're up and running. will be
 * followed by another mailbox message that carries the A9's virtual address
 * of the shared buffer. This would allow the A9's drivers to send virtual
 * addresses of the buffers.
 *
 * @RP_MSG_MBOX_STATE_CHANGE: informs the receiver that there is an inbound
 * message waiting in its own receive-side vring. please note that currently
 * this message is optional: alternatively, one can explicitly send the index
 * of the triggered virtqueue itself. the preferred approach will be decided
 * as we progress and experiment with those design ideas.
 *
 * @RP_MSG_MBOX_CRASH: this message indicates that the BIOS side is unhappy
 *
 * @RP_MBOX_ECHO_REQUEST: this message requests the remote processor to reply
 * with RP_MBOX_ECHO_REPLY
 *
 * @RP_MBOX_ECHO_REPLY: this is a reply that is sent when RP_MBOX_ECHO_REQUEST
 * is received.
 *
 * @RP_MBOX_ABORT_REQUEST:  tells the M3 to crash on demand
 */


//#define DIV_ROUND_UP(n,d)   (((n) + (d) - 1) / (d))
//#define RP_MSG_NUM_BUFS     (VQ0_SIZE) /* must be power of two */
#define RP_MSG_BUF_SIZE     (512)
//#define RP_MSG_BUFS_SPACE   (RP_MSG_NUM_BUFS * RP_MSG_BUF_SIZE * 2)

//#define PAGE_SIZE           (4096)
/*
 * The alignment to use between consumer and producer parts of vring.
 * Note: this is part of the "wire" protocol. If you change this, you need
 * to update your BIOS image as well
 */
//#define RP_MSG_VRING_ALIGN  (4096)

/* With 256 buffers, our vring will occupy 3 pages */
//#define RP_MSG_RING_SIZE    ((DIV_ROUND_UP(vring_size(RP_MSG_NUM_BUFS, \
//                            RP_MSG_VRING_ALIGN), PAGE_SIZE)) * PAGE_SIZE)

/* The total IPC space needed to communicate with a remote processor */
//#define RPMSG_IPC_MEM   (RP_MSG_BUFS_SPACE + 2 * RP_MSG_RING_SIZE)

#define ID_SYSM3_TO_A9      0
#define ID_A9_TO_SYSM3      1

#define ID_DSP_TO_A9        0
#define ID_A9_TO_DSP        1

#define CONSOLE_SYSM3_TO_A9 2
#define CONSOLE_A9_TO_SYSM3 3

#define CONSOLE_DSP_TO_A9   2
#define CONSOLE_A9_TO_DSP   3

#define ID_APPM3_TO_A9      200
#define ID_A9_TO_APPM3      201

typedef struct VirtQueueS2S_Object {
    /* Id for this VirtQueueS2S_Object */
    UInt16                  id;

    /* The function to call when buffers are consumed (can be NULL) */
    VirtQueueS2S_callback      callback;

    /* Shared state */
    struct vring            vring;

    /* Last available index; updated by VirtQueueS2S_getAvailBuf */
    UInt16                  last_avail_idx;

    /* Last available index; updated by VirtQueueS2S_addUsedBuf */
    UInt16                  last_used_idx;

    /* Will eventually be used to kick remote processor */
    UInt16                  procId;

    /* Address of the vring status variable in shared memory. */
    /* For TX vq, the address is for this proc's variable.    */
    /* For RX vq, the address is for the peer proc's variable.*/
    UInt8*					status;

	/* Indicates TX or RX direction */
    VirtQueueS2S_dir			direction;
} VirtQueueS2S_Object;

static struct VirtQueueS2S_Object *queueRegistry[MAX_VIRTQUEUES];


static inline Void * mapPAtoVA(UInt pa)
{

    UInt32 va;

    IpcMemory_physToVirt(pa, &va);
    return (void*)va;
}

static inline UInt mapVAtoPA(Void * va)
{
    UInt32 pa;

    IpcMemory_virtToPhys((UInt32)va, &pa);
    return pa;
}

/*!
 * ======== VirtQueueS2S_kick ========
 */
Void VirtQueueS2S_kick(VirtQueueS2S_Handle vq)
{
    /* For now, simply interrupt remote processor */
    if (vq->vring.avail->flags & VRING_AVAIL_F_NO_INTERRUPT) {
        Log_print0(Diags_USER1,
                "VirtQueueS2S_kick: no kick because of VRING_AVAIL_F_NO_INTERRUPT\n");
        return;
    }

    Log_print2(Diags_USER1,
            "VirtQueueS2S_kick: Sending interrupt to proc %d with payload 0x%x\n",
            (IArg)vq->procId, (IArg)vq->id);
    InterruptProxy_intSend(vq->procId, vq->id);
}


Void VirtQueueS2S_dump(VirtQueueS2S_Handle vq)
{
	System_printf("id %d: kick proc %d, last_avail_idx %d, last_used_idx %d, avail->idx %d, used->idx %d\n",
	vq->id, vq->procId, vq->last_avail_idx, vq->last_used_idx, vq->vring.avail->idx, vq->vring.used->idx);
}


/*!
 * ======== VirtQueueS2S_addUsedBuf ========
 */
Int VirtQueueS2S_addUsedBuf(VirtQueueS2S_Handle vq, Int16 head, int len)
{
    struct vring_used_elem *used;

    if ((head > vq->vring.num) || (head < 0)) {
        Error_raise(NULL, Error_E_generic, 0, 0);
    }

    /*
    * The virtqueue contains a ring of used buffers.  Get a pointer to the
    * next entry in that used ring.
    */
    used = &vq->vring.used->ring[vq->vring.used->idx % vq->vring.num];
    used->id = head;
    used->len = len;

    vq->vring.used->idx++;

    return (0);
}


/*!
 * ======== VirtQueueS2S_addAvailBuf ========
 */
Void VirtQueueS2S_addAvailBuf(VirtQueueS2S_Object *vq, Void *buf)
{
    UInt16 avail;

    avail =  vq->vring.avail->idx++ % vq->vring.num;

    vq->vring.desc[avail].addr = mapVAtoPA(buf);
    vq->vring.desc[avail].len = RP_MSG_BUF_SIZE;

    vq->vring.desc[avail].flags =2;
    vq->vring.avail->ring[avail] = avail;
}


/*!
 * ======== VirtQueueS2S_getUsedBuf ========
 */
Void *VirtQueueS2S_getUsedBuf(VirtQueueS2S_Object *vq)
{
    UInt16 head;
    Void *buf;

    /* There's nothing available? */
    if (vq->last_used_idx == vq->vring.used->idx) {
        return (NULL);
    }

    head = vq->vring.used->ring[vq->last_used_idx % vq->vring.num].id;
    vq->last_used_idx++;

    buf = mapPAtoVA(vq->vring.desc[head].addr);

    return (buf);
}


/*!
 * ======== VirtQueueS2S_getAvailBuf ========
 */
Int16 VirtQueueS2S_getAvailBuf(VirtQueueS2S_Handle vq, Void **buf, int *len)
{
    UInt16 head;

    Log_print6(Diags_USER1, "getAvailBuf vq: 0x%x %d %d %d 0x%x 0x%x\n",
	(IArg)vq,
        vq->last_avail_idx, vq->vring.avail->idx, vq->vring.num,
        (IArg)&vq->vring.avail, (IArg)vq->vring.avail);

    /* There's nothing available? */
    if (vq->last_avail_idx == vq->vring.avail->idx) {
        /* We need to know about added buffers */
        vq->vring.used->flags &= ~VRING_USED_F_NO_NOTIFY;

        return (-1);
    }

    /* TODO: think of adding the NO_NOTIFY optimization later on */
    /* vq->vring.used->flags |= VRING_USED_F_NO_NOTIFY; */

    /*
     * Grab the next descriptor number they're advertising, and increment
     * the index we've seen.
     */
    head = vq->vring.avail->ring[vq->last_avail_idx++ % vq->vring.num];

    *buf = mapPAtoVA(vq->vring.desc[head].addr);
    *len = vq->vring.desc[head].len;

    return (head);
}


/*!
 * ======== VirtQueueS2S_disableCallback ========
 */
Void VirtQueueS2S_disableCallback(VirtQueueS2S_Object *vq)
{
    Log_print0(Diags_USER1, "VirtQueueS2S_disableCallback called.");
}


/*!
 * ======== VirtQueueS2S_enableCallback ========
 */
Bool VirtQueueS2S_enableCallback(VirtQueueS2S_Object *vq)
{
    Log_print0(Diags_USER1, "VirtQueueS2S_enableCallback called.");

    //TODO
    return (FALSE);
}


/*!
 * ======== VirtQueueS2S_isr ========
 * Note 'arg' is ignored: it is the Hwi argument, not the mailbox argument.
 */
Void VirtQueueS2S_isr(UArg msg)
{
    VirtQueueS2S_Object *vq;

    Log_print1(Diags_USER1, "VirtQueueS2S_isr received msg = 0x%x\n", msg);

	vq = queueRegistry[msg];
	if (vq) {
		vq->callback(vq);
	}
}


/*!
 * ======== VirtQueueS2S_create ========
 */
VirtQueueS2S_Object *VirtQueueS2S_create(UInt32 vqId, UInt32 remoteprocId,
	VirtQueueS2S_callback callback, Vring_params *params,
	VirtQueueS2S_dir direction, UInt8 *status_addr)
{
    VirtQueueS2S_Object *vq;
    void *vring_phys;
    Error_Block eb;

   	if(vqId >= MAX_VIRTQUEUES) {
		Log_error1("VirtQueue Id exceeds maximum: %d", vqId);
		return NULL;
	}
	if(queueRegistry[vqId]) {
		Log_error1("VirtQueue Id already used: %d", vqId);
		return NULL;
	}

    Error_init(&eb);
    vq = Memory_alloc(NULL, sizeof(VirtQueueS2S_Object), 0, &eb);
    if (!vq) {
        return (NULL);
    }

    vq->callback = callback;
    vq->id = vqId;
    vq->procId = remoteprocId;
    vq->status = status_addr;
    vq->last_avail_idx = 0;
    vq->last_used_idx = 0;
    vq->direction = direction;

    Log_print3(Diags_USER1,
		"vring: id=%d addr=0x%x size=0x%x\n", vqId, params->addr,
		vring_size(params->num, params->align));

    vring_init(&(vq->vring), params->num, params->addr, params->align);

	/* Each processor clears only its TX vq memory. */
    if(direction = VirtQueueS2S_TX) {
		memset(params->addr, 0, vring_size(params->num, params->align));
		/* Don't trigger a mailbox message every time remote rpoc */
		/* makes another buffer available.                        */
		vq->vring.used->flags |= VRING_USED_F_NO_NOTIFY;
	}

    //if (master) {
        //memset(vring_phys, 0, RPMSG_IPC_MEM);
        //for (i = 0; i < RP_MSG_NUM_BUFS - 1; i++) {
            //vq->vring.desc[i].next = i + 1;
        //}

		///* set up the receive buffers */
        //for (i = 0; i < RP_MSG_NUM_BUFS; i++) {
            //buf = (void*) ((UInt)buf_addr + i * RP_MSG_BUF_SIZE);
            //VirtQueueS2S_addAvailBuf(vq, buf);
        //}
    //}

    queueRegistry[vqId] = vq;

    InterruptProxy_intRegister(remoteprocId, VirtQueueS2S_isr, remoteprocId);

    return (vq);
}


/*!
 * ======== VirtQueueS2S_get ========
 */
VirtQueueS2S_Object *VirtQueueS2S_get(UInt32 vqid)
{
    VirtQueueS2S_Object *vq = NULL;

	if(vqid < MAX_VIRTQUEUES)
		vq = queueRegistry[vqId];

	return vq;
}


/*!
 * ======== VirtQueueS2S_setCallback ========
 */
Int VirtQueueS2S_setCallback(UInt32 vqid, VirtQueueS2S_callback callback, UArg data)
{
    Int status = 1;
    VirtQueueS2S_Object *vq = VirtQueueS2S_get(UInt32 vqid);

	if(vq) {
		vq->callback = callback;
		status = 0;
	}

	return status;
}


Void VirtQueueS2S_prime(VirtQueueS2S_Object *vq, UInt16 remoteprocId, int vqid)
{
	/* fill the Available ring with buffers */
}


/* This array serves as a way to look up virtQueues when creating higher
 * level transports, such as MessageQcopy.  So far, the convention is
 * that the first pair of vrings defined for a remote proc will be
 * used for the primary MessageQcopy transport.
 *
 * Since none of this info is derived, it's all in the shared page, the
 * search could operate directly on the shared page instead of using
 * this array.
 */
static struct VirtioIPC_vqdev_db {
	UInt32 virtioId;
	UInt32 rank;	/* zero for primary... */
	UInt32 tx_vqId;
	UInt32 rx_vqId;
	UInt32 procId;
} VirtioIPC_vqdev_db[MAX_VQ_PAIRS];

static VirtioIPC_vqdev_cnt;


/* look up the requested VirtQueues */
/* rank is 0 for primary vqs        */
Bool VirtioIPC_getVirtQueues(UInt32 type, UInt32 procId, UInt32 rank, UInt32 *tx_vqId, UInt32 *rx_vqId)
{
	Int i;

	for(i=0; i<VirtioIPC_vqdev_cnt; i++) {
		if(type = VirtioIPC_vqdev_db[i].virtioId &&
		           rank = VirtioIPC_vqdev_db[i].rank &&
		           procId = VirtioIPC_vqdev_db[i].procId) {
			*tx_vqid = VirtioIPC_vqdev_db[i].tx_vqid;
			*rx_vqid = VirtioIPC_vqdev_db[i].rx_vqid;
			return 1;
		}
	}
	return 0;
}


/* Handles processing of one shared page.  Maybe called multiple times with
 * different shared pages. Each pair of vrings defined in the shared paged
 * will result in a TX/RX pair of VirtQueues being created.
 * One assumption made is that all vrings will be accessed as VirtQueues.
 * The VirtQueue callback to the higher level transport is set to
 * Null.  It must be changed when higher level transport is initialized.
 */
Int VirtioIPC_init(Void *shared_page)
{
	struct shared_ipc_page *sp = (struct shared_ipc_page *)shared_page;
	UInt32 sp_pos;		/* used to calcuate address of structs within the shared page */
	struct fw_rsc_evdev *vd;
	struct fw_rsc_vdev_vring *tx_vr, *rx_vr;
	struct fw_rsc_devmem2 *dm;
	Vring_params params;
	VirtQueueS2S_Handle tx_vq, rx_vq;
	UInt8 *tx_status, *rx_status;
	UInt32 procId;
	UInt32 num_vd, vd_cnt;
	UInt32 num_vr, vr_cnt;	/* counts pairs of vrings */

	/* check version of shared_ipc_page */
	if(sp->version != 1) {
		 Error_raise(NULL, Error_E_generic, "Unsupported shared_ipc_page version 0x%x", sp->version);
	}

	num_vd = sp->num;
	for(vd_cnt == 0; vd_cnt < num_vd; vd_cnt++) {
		vd = (struct fw_rsc_evdev *)((UInt32)shared_page + sp->offset[vd_cnt]);
		sp_pos = (UInt32)vd + sizeof(struct fw_rsc_evdev);

		// Check virtio_id to see what the type is.  Skip non-rpmsg?

		num_vr = vd->num_of_vrings;
		for(vr_cnt = 0; vr_cnt < num_vr; vr_cnt += 2) {
			if(vd->proc_id1 == Multiproc_self()) {
				tx_vr = (struct fw_rsc_vdev_vring *)sp_pos;
				rx_vr = (struct fw_rsc_vdev_vring *)(sp_pos + sizeof(struct fw_rsc_vdev_vring));
				tx_status = &vd->status1;
				rx_status = &vd->status2;
				procId = vd->rpoc_id2;
			} else {
				Assert_isTrue(vd->proc_id2 == Multiproc_self());

				rx_vr = (struct fw_rsc_vdev_vring *)sp_pos;
				tx_vr = (struct fw_rsc_vdev_vring *)(sp_pos + sizeof(struct fw_rsc_vdev_vring));
				rx_status = &vd->status1;
				tx_status = &vd->status2;
				procId = vd->rpoc_id1;
			}

			params.num = tx_vr->num;
			params.addr = tx_vr->da;
			params.align = tx_vr->align;
			tx_vq = VirtQueueS2S_Object *VirtQueueS2S_create(tx_vr->notify_id, procId,
					NULL, &params, VirtQueueS2S_TX, tx_status);

			if(tx_vq == NULL)
				Error_raise(NULL, Error_E_generic, "VirtQueue creation failure: %d", tx_vr->notify_id);

			params.num = rx_vr->num;
			params.addr = rx_vr->da;
			params.align = rx_vr->align;
			rx_vq = VirtQueueS2S_Object *VirtQueueS2S_create(rx_vr->notify_id, procId,
					NULL, &params, VirtQueueS2S_RX, rx_status);

			if(rx_vq == NULL)
				Error_raise(NULL, Error_E_generic, "VirtQueue creation failure: %d", rx_vr->notify_id);

			VirtioIPC_vqdev_db[VirtioIPC_VQ_cnt].virtioId = vd->vertio_id;
			VirtioIPC_vqdev_db[VirtioIPC_VQ_cnt].rank = vr_cnt;
			VirtioIPC_vqdev_db[VirtioIPC_VQ_cnt].tx_vqId = tx_vr->notify_id;
			VirtioIPC_vqdev_db[VirtioIPC_VQ_cnt].rx_vqId = rx_vr->notify_id;
			VirtioIPC_vqdev_db[VirtioIPC_VQ_cnt].procId = procId;
			VirtioIPC_vqdev_cnt++;
		}

		/* signal completion of the initialization of the TX vrings */
		if(vd->proc_id1 == Multiproc_self()) {
			vd->status1 = 1;
		} else {
			vd->status2 = 1;
		}

		/* Deal with any fw_rsc_devmem entries here */
	}
}



/*!
 * ======== VirtQueueS2S_startup ========
 */
Void VirtQueueS2S_startup()
{
	/* InterruptProxy_intRegister moved to VirtQueueS2S_create() */
}




