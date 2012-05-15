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
 *  @file       VirtQueue.h
 *
 *  @brief      Virtio Queue interface for BIOS
 *
 *  Differences between BIOS version and Linux kernel (include/linux/virtio.h):
 *  - Renamed module from virtio.h to VirtQueue.h to match the API prefixes;
 *  - BIOS (XDC) types and CamelCasing used;
 *  - virtio_device concept removed (i.e, assumes no containing device);
 *  - removed scatterlist;
 *  - VirtQueues are created statically here, so just added a VirtQueueS2S_init()
 *    fxn to take the place of the Virtio vring_new_virtqueue() API;
 *  - The notify function is implicit in the implementation, and not provided
 *    by the client, as it is in Linux virtio.
 *  - Broke into APIs to add/get used and avail buffers, as the API is
 *    assymmetric.
 *
 *  Usage:
 *     This IPC only works between one processor designated as the Host (Linux)
 *     and one or more Slave processors (BIOS).
 *
 *     For any Host/Slave pair, there are 2 VirtQueues (aka Vrings);
 *     Only the Host adds new buffers to the avail list of a vring;
 *     Available buffers can be empty or full, depending on direction;
 *     Used buffer means "processed" (emptied or filled);
 *
 *  Host:
 *    - To send buffer to the slave processor:
 *          add_avail_buf(slave_virtqueue);
 *          kick(slave_virtqueue);
 *          get_used_buf(slave_virtqueue);
 *    - To receive buffer from slave processor:
 *          add_avail_buf(host_virtqueue);
 *          kick(host_virtqueue);
 *          get_used_buf(host_virtqueue);
 *
 *  Slave:
 *    - To send buffer to the host:
 *          get_avail_buf(host_virtqueue);
 *          add_used_buf(host_virtqueue);
 *          kick(host_virtqueue);
 *    - To receive buffer from the host:
 *          get_avail_buf(slave_virtqueue);
 *          add_used_buf(slave_virtqueue);
 *          kick(slave_virtqueue);
 *
 *  All VirtQueue operations can be called in any context.
 *
 *  The virtio header should be included in an application as follows:
 *  @code
 *  #include <ti/ipc/rpmsg/VirtQueue.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef ti_ipc_VirtQueueS2S__include
#define ti_ipc_VirtQueueS2S__include

#if defined (__cplusplus)
extern "C" {
#endif

typedef enum VirtioIPC_vqdev_types {
	VirtioIPC_RPMSG = 1
} VirtioIPC_vqdev_types;

typedef enum VirtQueueS2S_dir {
	VirtQueueS2S_TX,
	VirtQueueS2S_RX
} VirtQueueS2S_dir;

typedef struct Vring_params {
	UInt32	num;
	UInt32	addr;
	UInt32	align;
} Vring_params;

/*!
 *  @brief  a queue to register buffers for sending or receiving.
 */
typedef struct VirtQueueS2S_Object *VirtQueueS2S_Handle;

/*!
 *  @var     VirtQueueS2S_callback
 *  @brief   Signature of any callback function that can be registered with the
 *           VirtQueue
 *
 *  @param[in]  VirtQueue     Pointer to the VirtQueue which was signalled.
 */
typedef Void (*VirtQueueS2S_callback)(UArg);

/*!
 *  @brief      Initialize at runtime the VirtQueue
 *
 *  @param[in]  callback  the clients callback function.
 *  @param[in]  procId    Processor ID associated with this VirtQueue.
 *
 *  @Returns    Returns a handle to a new initialized VirtQueue.
 */
VirtQueueS2S_Handle VirtQueueS2S_create(UInt32 vqId, UInt32 remoteprocId,
	VirtQueueS2S_callback callback, Vring_params *params,
	VirtQueueS2S_dir direction, UInt8 *status_addr);


/*!
 *  @brief      Notify other processor of new buffers in the queue.
 *
 *  After one or more add_buf calls, invoke this to kick the other side.
 *
 *  @param[in]  vq        the VirtQueue.
 *
 *  @sa         VirtQueueS2S_addBuf
 */
Void VirtQueueS2S_kick(VirtQueueS2S_Handle vq);

/*!
 *  @brief       Used at startup-time for initialization
 *
 *  Should be called before any other VirtQueue APIs
 */
Void VirtQueueS2S_startup();

Int VirtQueueS2S_setCallback(UInt32 vqid, VirtQueueS2S_callback callback, UArg priv);

VirtQueueS2S_Handle VirtQueueS2S_getHandle(UInt32 vqid);

Void VirtQueueS2S_prime(VirtQueueS2S_Handle vq, UInt16 remoteprocId, int vqid);

Bool VirtioIPC_getVirtQueues(UInt32 type, UInt32 procId, UInt32 rank, UInt32 *tx_vqId, UInt32 *rx_vqId);

Int VirtioIPC_init(Void *shared_page);


/*
 *  ============================================================================
 *  Host Only Functions:
 *  ============================================================================
 */

/*!
 *  @brief      Add available buffer to virtqueue's available buffer list.
 *              Only used by Host.
 *
 *  @param[in]  vq        the VirtQueue.
 *  @param[in]  buf      the buffer to be processed by the slave.
 *
 *  @return     Remaining capacity of queue or a negative error.
 *
 *  @sa         VirtQueueS2S_getUsedBuf
 */
Void VirtQueueS2S_addAvailBuf(VirtQueueS2S_Handle vq, Void *buf);

/*!
 *  @brief      Get the next used buffer.
 *              Only used by Host.
 *
 *  @param[in]  vq        the VirtQueue.
 *
 *  @return     Returns NULL or the processed buffer.
 *
 *  @sa         VirtQueueS2S_addAvailBuf
 */
Void *VirtQueueS2S_getUsedBuf(VirtQueueS2S_Handle vq);

/*
 *  ============================================================================
 *  Slave Only Functions:
 *  ============================================================================
 */

/*!
 *  @brief      Get the next available buffer.
 *              Only used by Slave.
 *
 *  @param[in]  vq        the VirtQueue.
 *  @param[out] buf       Pointer to location of available buffer;
 *
 *  @return     Returns a token used to identify the available buffer, to be
 *              passed back into VirtQueueS2S_addUsedBuf();
 *              token is negative if failure to find an available buffer.
 *
 *  @sa         VirtQueueS2S_addUsedBuf
 */
Int16 VirtQueueS2S_getAvailBuf(VirtQueueS2S_Handle vq, Void **buf, int *len);

/*!
 *  @brief      Add used buffer to virtqueue's used buffer list.
 *              Only used by Slave.
 *
 *  @param[in]  vq        the VirtQueue.
 *  @param[in]  token     token of the buffer to be added to vring used list.
 *
 *  @return     Remaining capacity of queue or a negative error.
 *
 *  @sa         VirtQueueS2S_getAvailBuf
 */
Int VirtQueueS2S_addUsedBuf(VirtQueueS2S_Handle vq, Int16 token, int len);

#define ID_SELF_TO_A9       0
#define ID_A9_TO_SELF       1

#define CONSOLE_SELF_TO_A9  2
#define CONSOLE_A9_TO_SELF  3

#define RP_MSG_BUF_SIZE     (512)

#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */
#endif /* ti_ipc_VirtQueueS2S__include */