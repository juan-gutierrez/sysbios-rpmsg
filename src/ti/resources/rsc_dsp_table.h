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
 *  ======== rsc_dsp_table.h ========
 *
 *  Include this table in each base image, which is read from remoteproc on
 *  host side.
 *
 *  These values are currently very OMAP4 specific!
 *
 */


#ifndef _RSC_DSP_TABLE_H_
#define _RSC_DSP_TABLE_H_



/* Ducati Memory Map: */
#define L4_44XX_BASE            0x4a000000

#define L4_PERIPHERAL_L4CFG     (L4_44XX_BASE)
#define DSP_PERIPHERAL_L4CFG    0x4A000000

#define L4_PERIPHERAL_L4PER     0x48000000
#define DSP_PERIPHERAL_L4PER    0x48000000

#define L4_PERIPHERAL_L4EMU     0x54000000
#define DSP_PERIPHERAL_L4EMU    0xB4000000

#define DSP_MEM_TEXT            0x20000000
#define DSP_MEM_DATA            0x80000000
#define DSP_MEM_IPC_VRING       0xA0000000
#define DSP_MEM_IPC_DATA        0x9F000000

#define TEXT_DA                 0x20000000
#define DATA_DA                 0x80000000

#define IPC_DA                  0xA0000000
#define IPC_PA                  0xA8800000

#define VRING0_DA               0xA0000000
#define VRING1_DA               0xA0004000
#define BUFS0_DA                0xA0040000
#define BUFS1_DA                0xA0080000


/*
 * sizes of the virtqueues (expressed in number of buffers supported,
 * and must be power of 2)
 */
#define VQ0_SIZE                256
#define VQ1_SIZE                256
#define VQ2_SIZE                256
#define VQ3_SIZE                256

/* Size constants must match those used on host: include/asm-generic/sizes.h */
#define SZ_1M                           0x00100000
#define SZ_2M                           0x00200000
#define SZ_4M                           0x00400000
#define SZ_8M                           0x00800000
#define SZ_16M                          0x01000000
#define SZ_32M                          0x02000000
#define SZ_64M                          0x04000000
#define SZ_128M                         0x08000000
#define SZ_256M                         0x10000000
#define SZ_512M                         0x20000000


/* virtio ids: keep in sync with the linux "include/linux/virtio_ids.h" */
#define VIRTIO_ID_RPMSG		7 /* virtio remote processor messaging */

/* Indices of rpmsg virtio features we support */
#define VIRTIO_RPMSG_F_NS	0 /* RP supports name service notifications */

/* flip up bits whose indices represent features we support */
#define DSP_C0_FEATURES         1

/* Resource info: Must match include/linux/remoteproc.h: */
#define TYPE_CARVEOUT    0
#define TYPE_DEVMEM      1
#define TYPE_TRACE       2
#define TYPE_VRING       3
#define TYPE_VIRTIO_DEV  4
#define TYPE_VIRTIO_CFG  5

struct resource {
    u32 type;
    u32 id;
    u32 da_low;       /* Device (Ducati virtual) Address */
    u32 da_high;
    u32 pa_low;       /* Physical Address */
    u32 pa_high;
    u32 len;
    u32 flags;
    u32 pad1;
    u32 pad2;
    u32 pad3;
    u32 pad4;
    char name[48];
};

extern char * xdc_runtime_SysMin_Module_State_0_outbuf__A;
#define TRACEBUFADDR (u32)&xdc_runtime_SysMin_Module_State_0_outbuf__A


#pragma DATA_SECTION(resources, ".resource_table")
#pragma DATA_ALIGN(resources, 4096)
struct resource resources[] = {
    /*
     * Virtio entries must come first.
     */
    { TYPE_VIRTIO_DEV,0,DSP_C0_FEATURES,0,0,0,0,VIRTIO_ID_RPMSG,0,0,0,0,"vdev:rpmsg"},
    { TYPE_VRING, 0, VRING0_DA, 0, 0, 0,VQ0_SIZE,0,0,0,0,0,"vring:dsp->mpu"},
    { TYPE_VRING, 1, VRING1_DA, 0, 0, 0,VQ1_SIZE,0,0,0,0,0,"vring:mpu->dsp"},

    /*
     * Contig Memory allocation entries must come after the virtio entries,
     * but before the reset of the gang.
     */
    { TYPE_CARVEOUT, 0, DATA_DA, 0, 0, 0, SZ_4M, 0,0,0,0,0, "DSP_MEM_DATA"},
    { TYPE_CARVEOUT, 0, TEXT_DA, 0, 0, 0, SZ_1M, 0, 0,0,0,0,"DSP_MEM_TEXT"},
    /*
     * Misc entries
     */
    { TYPE_TRACE, 0, TRACEBUFADDR,0,0,0, 0x8000, 0,0,0,0,0,"trace:dsp"},
    /*
     * IOMMU configuration entries
     */
    /* an evil hack that will be removed once the Linux DMA API is ready */
    { TYPE_DEVMEM, 0, IPC_DA, 0, IPC_PA, 0, SZ_1M, 0, 0,0,0,0,"DSP_MEM_IPC"},

    { TYPE_DEVMEM, 0, DSP_PERIPHERAL_L4CFG, 0, L4_PERIPHERAL_L4CFG, 0, SZ_16M,
       0, 0,0,0,0,"DSP_PERIPHERAL_L4CFG"},
    { TYPE_DEVMEM, 0, DSP_PERIPHERAL_L4PER, 0, L4_PERIPHERAL_L4PER, 0, SZ_16M,
       0,0,0,0,0,"DSP_PERIPHERAL_L4PER"},
};

#endif /* _RSC_DSP_TABLE_H_ */
