/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 *   redistributing this file, you may do so under either license.
 *
 *   GPL LICENSE SUMMARY
 *
 *   Copyright(c) 2015 Intel Corporation. All rights reserved.
 *   Copyright(c) 2017 T-Platforms. All Rights Reserved.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of version 2 of the GNU General Public License as
 *   published by the Free Software Foundation.
 *
 *   BSD LICENSE
 *
 *   Copyright(c) 2015 Intel Corporation. All rights reserved.
 *   Copyright(c) 2017 T-Platforms. All Rights Reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copy
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * PCIe NTB Perf Linux driver
 */

/*
 * How to use this tool, by example.
 *
 * Assuming $DBG_DIR is something like:
 * '/sys/kernel/debug/ntb_perf/0000:00:03.0'
 * Suppose aside from local device there is at least one remote device
 * connected to NTB with index 0.
 *-----------------------------------------------------------------------------
 * Eg: install driver with specified chunk/total orders and dma-enabled flag
 *
 * root@self# insmod ntb_perf.ko chunk_order=19 total_order=28 use_dma
 *-----------------------------------------------------------------------------
 * Eg: check NTB ports (index) and MW mapping information
 *
 * root@self# cat $DBG_DIR/info
 *-----------------------------------------------------------------------------
 * Eg: start performance test with peer (index 0) and get the test metrics
 *
 * root@self# echo 0 > $DBG_DIR/run
 * root@self# cat $DBG_DIR/run
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pci.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sizes.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/random.h>
#include <linux/ntb.h>

#include "ntb_device_lending.h" // KW_DB

#define DRIVER_NAME		"ntb_perf"
#define DRIVER_VERSION		"2.0"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR("Dave Jiang <dave.jiang@intel.com>");
MODULE_DESCRIPTION("PCIe NTB Performance Measurement Tool");

#if (SK_HYNIX == 1)
/* procfs */
#  include <linux/proc_fs.h>

#  define PROC_SKIO           "skio"
   static struct proc_dir_entry *proc_skio_root;
#else
#  define MAX_THREADS_CNT		32
#  define DEF_THREADS_CNT		1
#  define MAX_CHUNK_SIZE		SZ_1M
#  define MAX_CHUNK_ORDER		20 /* no larger than 1M */

#  define DMA_TRIES		100
#  define DMA_MDELAY		10

#  define MSG_TRIES		1000
#  define MSG_UDELAY_LOW		1000000
#  define MSG_UDELAY_HIGH		2000000

#  define PERF_BUF_LEN 1024
#endif  /* SK_HYNIX */

static unsigned long max_mw_size;
module_param(max_mw_size, ulong, 0644);
MODULE_PARM_DESC(max_mw_size, "Upper limit of memory window size");

static unsigned char chunk_order = 19; /* 512K */
module_param(chunk_order, byte, 0644);
MODULE_PARM_DESC(chunk_order, "Data chunk order [2^n] to transfer");

static unsigned char total_order = 30; /* 1G */
module_param(total_order, byte, 0644);
MODULE_PARM_DESC(total_order, "Total data order [2^n] to transfer");

static bool use_dma; /* default to 0 */
module_param(use_dma, bool, 0644);
MODULE_PARM_DESC(use_dma, "Use DMA engine to measure performance");

#if (SK_HYNIX == 0)
/*==============================================================================
 *                         Perf driver data definition
 *==============================================================================
 */

enum perf_cmd {
	PERF_CMD_INVAL = -1,/* invalid spad command */
	PERF_CMD_SSIZE = 0, /* send out buffer size */
	PERF_CMD_RSIZE = 1, /* recv in  buffer size */
	PERF_CMD_SXLAT = 2, /* send in  buffer xlat */
	PERF_CMD_RXLAT = 3, /* recv out buffer xlat */
	PERF_CMD_CLEAR = 4, /* clear allocated memory */
	PERF_STS_DONE  = 5, /* init is done */
	PERF_STS_LNKUP = 6, /* link up state flag */
#if (SK_HYNIX == 1)
    PERF_CMD_DEV_ADD =  7,  /* Lender  : Add a device to list for sharing */
    PERF_CMD_DEV_RM  =  8,  /* Lender  : Remove a device to list */
    PERF_CMD_DEV_LST =  9,  /* Borrower: Request available device list */
    PERF_CMD_DEV_BRW = 10,  /* Borrower: Request to borrow a device */
    PERF_CMD_DEV_RTN = 11,  /* Borrower: Return borrowed device */
    PERF_CMD_DL_STOP = 12,  /* Lender  : Stop device lending */
#endif
};

struct perf_ctx;

struct perf_peer {
	struct perf_ctx	*perf;
	int pidx;
	int gidx;

	/* Outbound MW params */
	u64 outbuf_xlat;
	resource_size_t outbuf_size;
	void __iomem *outbuf;
	phys_addr_t out_phys_addr;
	dma_addr_t dma_dst_addr;

	/* Inbound MW params */
	dma_addr_t inbuf_xlat;
	resource_size_t inbuf_size;
	void		*inbuf;

	/* NTB connection setup service */
	struct work_struct	service;
	unsigned long		sts;

	struct completion init_comp;
};
#define to_peer_service(__work) \
	container_of(__work, struct perf_peer, service)

struct perf_thread {
	struct perf_ctx *perf;
	int tidx;

	/* DMA-based test sync parameters */
	atomic_t dma_sync;
	wait_queue_head_t dma_wait;
	struct dma_chan *dma_chan;

	/* Data source and measured statistics */
	void *src;
	u64 copied;
	ktime_t duration;
	int status;
	struct work_struct work;
};
#define to_thread_work(__work) \
	container_of(__work, struct perf_thread, work)

struct perf_ctx {
	struct ntb_dev *ntb;

	/* Global device index and peers descriptors */
	int gidx;
	int pcnt;
	struct perf_peer *peers;

	/* Performance measuring work-threads interface */
	unsigned long busy_flag;
	wait_queue_head_t twait;
	atomic_t tsync;
	u8 tcnt;
	struct perf_peer *test_peer;
	struct perf_thread threads[MAX_THREADS_CNT];

	/* Scratchpad/Message IO operations */
	int (*cmd_send)(struct perf_peer *peer, enum perf_cmd cmd, u64 data);
	int (*cmd_recv)(struct perf_ctx *perf, int *pidx, enum perf_cmd *cmd,
			u64 *data);

	struct dentry * dbgfs_dir;
    dma_addr_t      dma_handle; // KW_DB
    void          * pBuf_virt;  // KW_DB
};

/*
 * Scratchpads-base commands interface
 */
#define PERF_SPAD_CNT(_pcnt) \
	(3*((_pcnt) + 1))
#define PERF_SPAD_CMD(_gidx) \
	(3*(_gidx))
#define PERF_SPAD_LDATA(_gidx) \
	(3*(_gidx) + 1)
#define PERF_SPAD_HDATA(_gidx) \
	(3*(_gidx) + 2)
#define PERF_SPAD_NOTIFY(_gidx) \
	(BIT_ULL(_gidx))

/*
 * Messages-base commands interface
 */
#define PERF_MSG_CNT		3
#define PERF_MSG_CMD		0
#define PERF_MSG_LDATA		1
#define PERF_MSG_HDATA		2

#endif  /* SK_HYNIX */

/*==============================================================================
 *                           Static data declarations
 *==============================================================================
 */

static struct dentry *perf_dbgfs_topdir;

static struct workqueue_struct *perf_wq __read_mostly;

/*==============================================================================
 *                  NTB cross-link commands execution service
 *==============================================================================
 */

static void perf_terminate_test(struct perf_ctx *perf);

static inline bool perf_link_is_up(struct perf_peer *peer)
{
	u64 link;

	link = ntb_link_is_up(peer->perf->ntb, NULL, NULL);
	return !!(link & BIT_ULL_MASK(peer->pidx));
}

#include <linux/delay.h>   // KW_DB

static int perf_spad_cmd_send(struct perf_peer *peer, enum perf_cmd cmd,
			      u64 data)
{
	struct perf_ctx *perf = peer->perf;
	int try;
	u32 sts;

#if 0   // Hope to resolve "BUG: scheduling while atomic" issue
	dev_dbg(&perf->ntb->dev, "CMD send: %d 0x%llx\n", cmd, data);
#endif
    dev_info(&perf->ntb->dev, "SPAD CMD send: %d 0x%llx\n", cmd, data);  // KW_DB


	/*
	 * Perform predefined number of attempts before give up.
	 * We are sending the data to the port specific scratchpad, so
	 * to prevent a multi-port access race-condition. Additionally
	 * there is no need in local locking since only thread-safe
	 * service work is using this method.
	 */
	for (try = 0; try < MSG_TRIES; try++) {
		if (!perf_link_is_up(peer))
        {   // KW_DB
            dev_err(&perf->ntb->dev, "!perf_link_is_up\n");    // KW_DB
			return -ENOLINK;
        }   // KW_DB

		sts = ntb_peer_spad_read(perf->ntb, peer->pidx,
					 PERF_SPAD_CMD(perf->gidx));
		if (sts != PERF_CMD_INVAL) {
            if (cmd != PERF_CMD_CFG_WR)
            {
                dev_warn(&perf->ntb->dev, "Calling usleep_range()\n");    // KW_DB
                usleep_range(MSG_UDELAY_LOW, MSG_UDELAY_HIGH);
            }
            else
            {
                ///dev_info(&perf->ntb->dev, "Calling udelay()\n");    // KW_DB
                mdelay(MSG_UDELAY_LOW/100000);
            }
			continue;
		}

		ntb_peer_spad_write(perf->ntb, peer->pidx,
				    PERF_SPAD_LDATA(perf->gidx),
				    lower_32_bits(data));
		ntb_peer_spad_write(perf->ntb, peer->pidx,
				    PERF_SPAD_HDATA(perf->gidx),
				    upper_32_bits(data));
		ntb_peer_spad_write(perf->ntb, peer->pidx,
				    PERF_SPAD_CMD(perf->gidx),
				    cmd);
		ntb_peer_db_set(perf->ntb, PERF_SPAD_NOTIFY(peer->gidx));
#if 0   // Hope to resolve "BUG: scheduling while atomic" issue
		dev_dbg(&perf->ntb->dev, "DB ring peer %#llx\n",
			PERF_SPAD_NOTIFY(peer->gidx));
#endif
		break;
	}

    dev_info(&perf->ntb->dev, "try[%d] MSG_TRIES[%d]\n", try,MSG_TRIES);    // KW_DB
	return try < MSG_TRIES ? 0 : -EAGAIN;
}

static int perf_spad_cmd_recv(struct perf_ctx *perf, int *pidx,
			      enum perf_cmd *cmd, u64 *data)
{
	struct perf_peer *peer;
	u32 val;

    printk("%s(): Entered. (Ln %d)\n", __func__, __LINE__); // KW_DB
	ntb_db_clear(perf->ntb, PERF_SPAD_NOTIFY(perf->gidx));

	/*
	 * We start scanning all over, since cleared DB may have been set
	 * by any peer. Yes, it makes peer with smaller index being
	 * serviced with greater priority, but it's convenient for spad
	 * and message code unification and simplicity.
	 */
	for (*pidx = 0; *pidx < perf->pcnt; (*pidx)++) {
        int cntr=0;// KW_DB
		peer = &perf->peers[*pidx];


        while (!perf_link_is_up(peer)) // KW_DB
        {
            if(cntr >=100)
                break;
            printk("%s(): Delaying 300ms. (Ln %d)\n", __func__, __LINE__); // KW_DB
            mdelay(300); // KW_DB
            cntr++;
        }

		if (!perf_link_is_up(peer))
        {
            printk("%s(): Continuing. (Ln %d)\n", __func__, __LINE__); // KW_DB
			continue;
        }

		val = ntb_spad_read(perf->ntb, PERF_SPAD_CMD(peer->gidx));
		if (val == PERF_CMD_INVAL)
        {
            printk("%s(): Continuing. (Ln %d)\n", __func__, __LINE__); // KW_DB
			continue;
        }

		*cmd = val;

		val = ntb_spad_read(perf->ntb, PERF_SPAD_LDATA(peer->gidx));
		*data = val;

		val = ntb_spad_read(perf->ntb, PERF_SPAD_HDATA(peer->gidx));
		*data |= (u64)val << 32;

		/* Next command can be retrieved from now */
		ntb_spad_write(perf->ntb, PERF_SPAD_CMD(peer->gidx),
			       PERF_CMD_INVAL);

		dev_dbg(&perf->ntb->dev, "CMD recv: %d 0x%llx\n", *cmd, *data);

        printk("%s(): Returning 0. (Ln %d)\n", __func__, __LINE__); // KW_DB
		return 0;
	}

    printk("%s(): Returning -ENODATA. (Ln %d)\n", __func__, __LINE__); // KW_DB
	return -ENODATA;
}

static int perf_msg_cmd_send(struct perf_peer *peer, enum perf_cmd cmd,
			     u64 data)
{
	struct perf_ctx *perf = peer->perf;
	int try, ret;
	u64 outbits;

	dev_dbg(&perf->ntb->dev, "CMD send: %d 0x%llx\n", cmd, data);
    dev_info(&perf->ntb->dev, "MSG CMD send: %d 0x%llx\n", cmd, data);  // KW_DB

	/*
	 * Perform predefined number of attempts before give up. Message
	 * registers are free of race-condition problem when accessed
	 * from different ports, so we don't need splitting registers
	 * by global device index. We also won't have local locking,
	 * since the method is used from service work only.
	 */
	outbits = ntb_msg_outbits(perf->ntb);
	for (try = 0; try < MSG_TRIES; try++) {
		if (!perf_link_is_up(peer))
			return -ENOLINK;

		ret = ntb_msg_clear_sts(perf->ntb, outbits);
		if (ret)
			return ret;

		ntb_peer_msg_write(perf->ntb, peer->pidx, PERF_MSG_LDATA,
				   lower_32_bits(data));

		if (ntb_msg_read_sts(perf->ntb) & outbits) {
			usleep_range(MSG_UDELAY_LOW, MSG_UDELAY_HIGH);
			continue;
		}

		ntb_peer_msg_write(perf->ntb, peer->pidx, PERF_MSG_HDATA,
				   upper_32_bits(data));

		/* This call shall trigger peer message event */
		ntb_peer_msg_write(perf->ntb, peer->pidx, PERF_MSG_CMD, cmd);

		break;
	}

	return try < MSG_TRIES ? 0 : -EAGAIN;
}

static int perf_msg_cmd_recv(struct perf_ctx *perf, int *pidx,
			     enum perf_cmd *cmd, u64 *data)
{
	u64 inbits;
	u32 val;

	inbits = ntb_msg_inbits(perf->ntb);

	if (hweight64(ntb_msg_read_sts(perf->ntb) & inbits) < 3)
		return -ENODATA;

	val = ntb_msg_read(perf->ntb, pidx, PERF_MSG_CMD);
	*cmd = val;

	val = ntb_msg_read(perf->ntb, pidx, PERF_MSG_LDATA);
	*data = val;

	val = ntb_msg_read(perf->ntb, pidx, PERF_MSG_HDATA);
	*data |= (u64)val << 32;

	/* Next command can be retrieved from now */
	ntb_msg_clear_sts(perf->ntb, inbits);

	dev_dbg(&perf->ntb->dev, "CMD recv: %d 0x%llx\n", *cmd, *data);

	return 0;
}

static int perf_cmd_send(struct perf_peer *peer, enum perf_cmd cmd, u64 data)
{
	struct perf_ctx *perf = peer->perf;

    printk("%s(): cmd[x%X] (Ln %d)\n", __func__, cmd, __LINE__); // KW_DB
#if (SK_HYNIX == 0)
	if (cmd == PERF_CMD_SSIZE || cmd == PERF_CMD_SXLAT)
#endif
		return perf->cmd_send(peer, cmd, data);

	dev_err(&perf->ntb->dev, "Send invalid command\n");
	return -EINVAL;
}

#if (SK_HYNIX == 1)
/**
 *
 */
int lender_cmd_send(struct perf_peer *peer, enum perf_cmd cmd, u64 data)
{
    return perf_cmd_send(peer, cmd, data);
}

/**
 * @param   data: Bits 63-60: data size
 *                Bits 59-48: cmd idx
 *                Bits 47-32: config space offset
 *                Bits 31-00: data
 */
int borrower_cmd_send(struct perf_peer *peer, enum perf_cmd cmd, u64 data)
{
    return perf_cmd_send(peer, cmd, data);
}
#endif

static int perf_cmd_exec(struct perf_peer *peer, enum perf_cmd cmd)
{
    printk("%s(): cmd[x%X] (Ln %d)\n", __func__, cmd, __LINE__); // KW_DB
	switch (cmd) {
	case PERF_CMD_SSIZE:
	case PERF_CMD_RSIZE:
	case PERF_CMD_SXLAT:
	case PERF_CMD_RXLAT:
	case PERF_CMD_CLEAR:
#if (SK_HYNIX == 1)
    case PERF_CMD_DEV_ADD:
    case PERF_CMD_DEV_RM:
    case PERF_CMD_DEV_LST:
    case PERF_CMD_DEV_BRW:
    case PERF_CMD_DEV_RTN:
    case PERF_CMD_DL_STOP:
    case PERF_CMD_DEV_REG:
    case PERF_CMD_DNB_ADR:
    case PERF_CMD_MW0_ADR:  // KW_DB_20241211
    case PERF_CMD_CFG_RD:
    case PERF_CMD_CFG_WR:
#endif
		break;
	default:
		dev_err(&peer->perf->ntb->dev, "Exec invalid command\n");
		return -EINVAL;
	}

	/* No need of memory barrier, since bit ops have invernal lock */
	set_bit(cmd, &peer->sts);

	dev_dbg(&peer->perf->ntb->dev, "CMD exec: %d\n", cmd);

	(void)queue_work(system_highpri_wq, &peer->service);

	return 0;
}

static int perf_cmd_recv(struct perf_ctx *perf)
{
	struct perf_peer *peer;
	int ret, pidx, cmd;
	u64 data;

    printk("%s(): Entered. (Ln %d)\n", __func__, __LINE__); // KW_DB


	while (!(ret = perf->cmd_recv(perf, &pidx, &cmd, &data))) {
		peer = &perf->peers[pidx];

        printk("%s(): cmd[x%X] pidx[%d] (Ln %d)\n", __func__, cmd, pidx, __LINE__); // KW_DB
        if (!perf_link_is_up(peer)) // KW_DB
        {
            ///printk("%s(): Delaying. (Ln %d)\n", __func__, __LINE__); // KW_DB
            mdelay(10); // KW_DB
        }
		switch (cmd) {
		case PERF_CMD_SSIZE:
			peer->inbuf_size = data;
            printk("%s(): PERF_CMD_SSIZE. Calling perf_cmd_exec(peer, PERF_CMD_RSIZE) inbuf_size[x%llX]  (Ln %d)\n",
                __func__, peer->inbuf_size, __LINE__); // KW_DB
			return perf_cmd_exec(peer, PERF_CMD_RSIZE);
		case PERF_CMD_SXLAT:
			peer->outbuf_xlat = data;
            printk("%s(): PERF_CMD_SXLAT. Calling perf_cmd_exec(peer, PERF_CMD_RXLAT) outbuf_xlat[x%llX] (Ln %d)\n",
                __func__, peer->outbuf_xlat, __LINE__); // KW_DB
			return perf_cmd_exec(peer, PERF_CMD_RXLAT);
#if (SK_HYNIX == 1) // KW_DB
        case PERF_CMD_DEV_ADD:   // KW_DB
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_AD\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DEV_ADD);
        case PERF_CMD_DEV_RM:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_RM\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DEV_RM);
        case PERF_CMD_DEV_LST:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_LST\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DEV_LST);
        case PERF_CMD_DEV_BRW:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_BRW\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DEV_BRW);
        case PERF_CMD_DEV_RTN:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_RTN\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DEV_RTN);
        case PERF_CMD_DL_STOP:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DL_STOP\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DL_STOP);
        case PERF_CMD_DEV_REG:
            peer->dev_reg_addr = data;
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_REG\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DEV_REG);
        case PERF_CMD_DNB_ADR:
            peer->dnb_addr = data;
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DNB_ADR\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_DNB_ADR);
        // KW_DB_20241211->
        case PERF_CMD_MW0_ADR:
            peer->mw0_addr = data;
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_MW0_ADR\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_MW0_ADR);
        // KW_DB_20241211<-
        case PERF_CMD_CFG_RD:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_CFG_RD\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_CFG_RD);        
        case PERF_CMD_CFG_WR:
            dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_CFG_WR\n", __func__);
            return perf_cmd_exec(peer, PERF_CMD_CFG_WR); 
#endif
		default:
			dev_err(&perf->ntb->dev, "Recv invalid command\n");
			return -EINVAL;
		}
	}

	/* Return 0 if no data left to process, otherwise an error */
	return ret == -ENODATA ? 0 : ret;
}

static void perf_link_event(void *ctx)
{
	struct perf_ctx *perf = ctx;
	struct perf_peer *peer;
	bool lnk_up;
	int pidx;

	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		peer = &perf->peers[pidx];

		lnk_up = perf_link_is_up(peer);

        printk("%s(): sts[x%lX] (Ln %d)\n", __func__, peer->sts, __LINE__); // KW_DB
		if (lnk_up &&
		    !test_and_set_bit(PERF_STS_LNKUP, &peer->sts)) {
			perf_cmd_exec(peer, PERF_CMD_SSIZE);
		} else if (!lnk_up &&
			   test_and_clear_bit(PERF_STS_LNKUP, &peer->sts)) {
			perf_cmd_exec(peer, PERF_CMD_CLEAR);
		}
	}
}

static void perf_db_event(void *ctx, int vec)
{
	struct perf_ctx *perf = ctx;

	dev_dbg(&perf->ntb->dev, "DB vec %d mask %#llx bits %#llx\n", vec,
		ntb_db_vector_mask(perf->ntb, vec), ntb_db_read(perf->ntb));

	/* Just receive all available commands */
	(void)perf_cmd_recv(perf);
    printk("%s(): Leaving (Ln %d)\n", __func__, __LINE__); // KW_DB
}

static void perf_msg_event(void *ctx)
{
	struct perf_ctx *perf = ctx;

	dev_dbg(&perf->ntb->dev, "Msg status bits %#llx\n",
		ntb_msg_read_sts(perf->ntb));

	/* Messages are only sent one-by-one */
	(void)perf_cmd_recv(perf);
    printk("%s(): Leaving (Ln %d)\n", __func__, __LINE__); // KW_DB
}

static const struct ntb_ctx_ops perf_ops = {
	.link_event = perf_link_event,
	.db_event = perf_db_event,
	.msg_event = perf_msg_event
};

static void perf_free_outbuf(struct perf_peer *peer)
{
	(void)ntb_peer_mw_clear_trans(peer->perf->ntb, peer->pidx, peer->gidx);
}

static int perf_setup_outbuf(struct perf_peer *peer)
{
	struct perf_ctx *perf = peer->perf;
	int ret;

    dev_info(&perf->ntb->dev, "%s(): outbuf_xlat[x%llX] outbuf_size[%lld] (Ln %d)\n",
        __func__, peer->outbuf_xlat, peer->outbuf_size, __LINE__); // KW_DB

    // KW_DB->
    if (perf->ntb->ops->peer_mw_set_trans)
        dev_info(&perf->ntb->dev, "%s(): peer_mw_set_trans is not NULL. (Ln %d)\n",
        __func__, __LINE__); // KW_DB
    else
        dev_info(&perf->ntb->dev, "%s(): peer_mw_set_trans is NULL. (Ln %d)\n",
        __func__, __LINE__); // KW_DB
    // KW_DB<-
	/* Outbuf size can be unaligned due to custom max_mw_size */
	ret = ntb_peer_mw_set_trans(perf->ntb, peer->pidx, peer->gidx,
				    peer->outbuf_xlat, peer->outbuf_size);
	if (ret) {
		dev_err(&perf->ntb->dev, "Failed to set outbuf translation\n");
		return ret;
	}

	/* Initialization is finally done */
	set_bit(PERF_STS_DONE, &peer->sts);
	complete_all(&peer->init_comp);

	return 0;
}

static void perf_free_inbuf(struct perf_peer *peer)
{
	if (!peer->inbuf)
		return;

    dev_info(&peer->perf->ntb->dev, "%s(): inbuf DW0: 0x%X (Ln %d)\n",
        __func__, *((u32 *) peer->inbuf), __LINE__); // KW_DB

	(void)ntb_mw_clear_trans(peer->perf->ntb, peer->pidx, peer->gidx);
	dma_free_coherent(&peer->perf->ntb->pdev->dev, peer->inbuf_size,
			  peer->inbuf, peer->inbuf_xlat);
	peer->inbuf = NULL;
}

static int perf_setup_inbuf(struct perf_peer *peer)
{
	resource_size_t xlat_align, size_align, size_max;
	struct perf_ctx *perf = peer->perf;
	int ret;

	/* Get inbound MW parameters */
    dev_info(&perf->ntb->dev, "%s(): Calling ntb_mw_get_align() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	ret = ntb_mw_get_align(perf->ntb, peer->pidx, perf->gidx,
			       &xlat_align, &size_align, &size_max);
	if (ret) {
		dev_err(&perf->ntb->dev, "Couldn't get inbuf restrictions\n");
		return ret;
	}

	if (peer->inbuf_size > size_max) {
		dev_err(&perf->ntb->dev, "Too big inbuf size %pa > %pa\n",
			&peer->inbuf_size, &size_max);
		return -EINVAL;
	}

    dev_info(&perf->ntb->dev, "%s(): xlat_align[x%llX] size_align[x%llX] size_max[x%llX] (Ln %d)\n",
        __func__, (u64) xlat_align, (u64) size_align, (u64) size_max, __LINE__); // KW_DB

	peer->inbuf_size = round_up(peer->inbuf_size, size_align);

	perf_free_inbuf(peer);

	peer->inbuf = dma_alloc_coherent(&perf->ntb->pdev->dev,
					 peer->inbuf_size, &peer->inbuf_xlat,
					 GFP_KERNEL);

    dev_info(&perf->ntb->dev, "%s(): inbuf(%p) inbuf_size[%lld] (Ln %d)\n",
        __func__, peer->inbuf, peer->inbuf_size, __LINE__); // KW_DB

    dev_info(&perf->ntb->dev, "%s(): phy may be [0x%llX] (Ln %d)\n",
        __func__, virt_to_phys(peer->inbuf), __LINE__); // KW_DB



    ///dev_info(&perf->ntb->dev, "%s(): pAddr[0x%x] (Ln %d)\n",
    ///    __func__, dma_to_phys(&perf->ntb->pdev->dev, peer->inbuf), __LINE__); // KW_DB

	if (!peer->inbuf) {
		dev_err(&perf->ntb->dev, "Failed to alloc inbuf of %pa\n",
			&peer->inbuf_size);
		return -ENOMEM;
	}
	if (!IS_ALIGNED(peer->inbuf_xlat, xlat_align)) {
		dev_err(&perf->ntb->dev, "Unaligned inbuf allocated\n");
		goto err_free_inbuf;
	}

    dev_info(&perf->ntb->dev, "%s(): inbuf_xlat[x%llX] inbuf_size[%lld] (Ln %d)\n",
        __func__, (u64) peer->inbuf_xlat, peer->inbuf_size, __LINE__); // KW_DB
	ret = ntb_mw_set_trans(perf->ntb, peer->pidx, peer->gidx,
        peer->inbuf_xlat, peer->inbuf_size); /// KW_DB
	///ret = ntb_mw_set_trans(perf->ntb, peer->pidx, 4,
	///		       peer->inbuf_xlat, peer->inbuf_size); // KW_DB
    ///*(u32*)(peer->inbuf) = 0x20201010;  // KW_DB
    ///memset(peer->inbuf, 0x00, peer->inbuf_size); // KW_DB
	if (ret) {
		dev_err(&perf->ntb->dev, "Failed to set inbuf translation\n");
		goto err_free_inbuf;
	}

	/*
	 * We submit inbuf xlat transmission cmd for execution here to follow
	 * the code architecture, even though this method is called from service
	 * work itself so the command will be executed right after it returns.
	 */
    printk("%s(): exec PERF_CMD_SXLAT (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	(void)perf_cmd_exec(peer, PERF_CMD_SXLAT);

	return 0;

err_free_inbuf:
	perf_free_inbuf(peer);

	return ret;
}

static void perf_service_work(struct work_struct *work)
{
	struct perf_peer *peer = to_peer_service(work);

    // KW_DB_20250407->
    /*
     * Try to resolve the following symtom on Lender's log:
     *    switchtech_ntb_part_op_no_retry(): -EAGAIN (ln 218)
     */
    mdelay(1000);
    // KW_DB_20250407<-
    printk("%s(): sts[x%lX] (Ln %d)\n",
        __func__, peer->sts, __LINE__); // KW_DB

	if (test_and_clear_bit(PERF_CMD_SSIZE, &peer->sts))
    {
        printk("%s(): PERF_CMD_SSIZE. Calling perf_cmd_send(PERF_CMD_SSIZE, outbuf_size[x%llX]) (Ln %d)\n",
            __func__, (u64) peer->outbuf_size, __LINE__); // KW_DB
		perf_cmd_send(peer, PERF_CMD_SSIZE, peer->outbuf_size);
    }

	if (test_and_clear_bit(PERF_CMD_RSIZE, &peer->sts))
    {
        printk("%s(): PERF_CMD_RSIZE. Calling perf_setup_inbuf() (Ln %d)\n",
            __func__, __LINE__); // KW_DB
		perf_setup_inbuf(peer);
    }

	if (test_and_clear_bit(PERF_CMD_SXLAT, &peer->sts))
    {
        printk("%s(): PERF_CMD_SXLAT. Calling perf_cmd_send(PERF_CMD_SXLAT, inbuf_xlat[x%llX]) (Ln %d)\n",
            __func__, (u64) peer->inbuf_xlat, __LINE__); // KW_DB
		perf_cmd_send(peer, PERF_CMD_SXLAT, peer->inbuf_xlat);
    }

	if (test_and_clear_bit(PERF_CMD_RXLAT, &peer->sts))
    {
        printk("%s(): PERF_CMD_RXLAT. Calling perf_setup_outbuf() (Ln %d)\n",
            __func__, __LINE__); // KW_DB
		perf_setup_outbuf(peer);
    }

    printk("%s(): sts[x%lX] (Ln %d)\n",
        __func__, peer->sts, __LINE__); // KW_DB
	if (test_and_clear_bit(PERF_CMD_CLEAR, &peer->sts))
    {
        printk("%s(): PERF_CMD_CLEAR. Calling init_completion() (Ln %d)\n",
            __func__, __LINE__); // KW_DB
		init_completion(&peer->init_comp);
		clear_bit(PERF_STS_DONE, &peer->sts);
		if (test_bit(0, &peer->perf->busy_flag) &&
		    peer == peer->perf->test_peer) {
			dev_warn(&peer->perf->ntb->dev,
				"Freeing while test on-fly\n");
			perf_terminate_test(peer->perf);
		}
		perf_free_outbuf(peer);
		perf_free_inbuf(peer);
	}
#if (SK_HYNIX == 1)
    if (test_and_clear_bit(PERF_CMD_DEV_ADD, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_ADD\n", __func__);
    }

    if (test_and_clear_bit(PERF_CMD_DEV_RM, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_RM\n", __func__);
    }

    if (test_and_clear_bit(PERF_CMD_DEV_LST, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_LST\n", __func__);
    }

    if (test_and_clear_bit(PERF_CMD_DEV_BRW, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_BRW\n", __func__);
    }

    if (test_and_clear_bit(PERF_CMD_DEV_RTN, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_RTN\n", __func__);
    }

    if (test_and_clear_bit(PERF_CMD_DL_STOP, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DL_STOP\n", __func__);
    }

    if (test_and_clear_bit(PERF_CMD_DEV_REG, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DEV_REG\n", __func__);
        borrower_connect_remote_device(0);
    }

    if (test_and_clear_bit(PERF_CMD_DNB_ADR, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_DNB_ADR\n", __func__);
        ///borrower_set_dev_NTB_ADDR();
    }
    
    // KW_DB_20241211->
    if (test_and_clear_bit(PERF_CMD_MW0_ADR, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_MW0_ADR\n", __func__);
    }
    // KW_DB_20241211<-
    
    if (test_and_clear_bit(PERF_CMD_CFG_RD, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_CFG_RD\n", __func__);
    }
    
    if (test_and_clear_bit(PERF_CMD_CFG_WR, &peer->sts))
    {
        dev_dbg(&peer->perf->ntb->dev, "%s(): PERF_CMD_CFG_WR\n", __func__);
    }
#endif
}

static int perf_init_service(struct perf_ctx *perf)
{
	u64 mask;

	if (ntb_peer_mw_count(perf->ntb) < perf->pcnt) {
		dev_err(&perf->ntb->dev, "Not enough memory windows\n");
		return -EINVAL;
	}

	if (ntb_msg_count(perf->ntb) >= PERF_MSG_CNT) {
		perf->cmd_send = perf_msg_cmd_send;
		perf->cmd_recv = perf_msg_cmd_recv;

		dev_dbg(&perf->ntb->dev, "Message service initialized\n");

		return 0;
	}

	dev_dbg(&perf->ntb->dev, "Message service unsupported\n");

	mask = GENMASK_ULL(perf->pcnt, 0);
	if (ntb_spad_count(perf->ntb) >= PERF_SPAD_CNT(perf->pcnt) &&
	    (ntb_db_valid_mask(perf->ntb) & mask) == mask) {
		perf->cmd_send = perf_spad_cmd_send;
		perf->cmd_recv = perf_spad_cmd_recv;

		dev_dbg(&perf->ntb->dev, "Scratchpad service initialized\n");

		return 0;
	}

	dev_dbg(&perf->ntb->dev, "Scratchpad service unsupported\n");

	dev_err(&perf->ntb->dev, "Command services unsupported\n");

	return -EINVAL;
}

static int perf_enable_service(struct perf_ctx *perf)
{
	u64 mask, incmd_bit;
	int ret, sidx, scnt;

	mask = ntb_db_valid_mask(perf->ntb);
	(void)ntb_db_set_mask(perf->ntb, mask);

	ret = ntb_set_ctx(perf->ntb, perf, &perf_ops);
	if (ret)
		return ret;

	if (perf->cmd_send == perf_msg_cmd_send) {
		u64 inbits, outbits;

		inbits = ntb_msg_inbits(perf->ntb);
		outbits = ntb_msg_outbits(perf->ntb);
		(void)ntb_msg_set_mask(perf->ntb, inbits | outbits);

		incmd_bit = BIT_ULL(__ffs64(inbits));
		ret = ntb_msg_clear_mask(perf->ntb, incmd_bit);

		dev_dbg(&perf->ntb->dev, "MSG sts unmasked %#llx\n", incmd_bit);
	} else {
		scnt = ntb_spad_count(perf->ntb);
		for (sidx = 0; sidx < scnt; sidx++)
			ntb_spad_write(perf->ntb, sidx, PERF_CMD_INVAL);
		incmd_bit = PERF_SPAD_NOTIFY(perf->gidx);
		ret = ntb_db_clear_mask(perf->ntb, incmd_bit);

		dev_dbg(&perf->ntb->dev, "DB bits unmasked %#llx\n", incmd_bit);
	}
	if (ret) {
		ntb_clear_ctx(perf->ntb);
		return ret;
	}

	ntb_link_enable(perf->ntb, NTB_SPEED_AUTO, NTB_WIDTH_AUTO);
	/* Might be not necessary */
	ntb_link_event(perf->ntb);

	return 0;
}

static void perf_disable_service(struct perf_ctx *perf)
{
	int pidx;

	if (perf->cmd_send == perf_msg_cmd_send) {
		u64 inbits;

		inbits = ntb_msg_inbits(perf->ntb);
		(void)ntb_msg_set_mask(perf->ntb, inbits);
	} else {
		(void)ntb_db_set_mask(perf->ntb, PERF_SPAD_NOTIFY(perf->gidx));
	}

	ntb_clear_ctx(perf->ntb);

	for (pidx = 0; pidx < perf->pcnt; pidx++)
		perf_cmd_exec(&perf->peers[pidx], PERF_CMD_CLEAR);

	for (pidx = 0; pidx < perf->pcnt; pidx++)
		flush_work(&perf->peers[pidx].service);

	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		struct perf_peer *peer = &perf->peers[pidx];

		ntb_spad_write(perf->ntb, PERF_SPAD_CMD(peer->gidx), 0);
	}

	ntb_db_clear(perf->ntb, PERF_SPAD_NOTIFY(perf->gidx));

	ntb_link_disable(perf->ntb);
}

/*==============================================================================
 *                      Performance measuring work-thread
 *==============================================================================
 */

static void perf_dma_copy_callback(void *data)
{
	struct perf_thread *pthr = data;

	atomic_dec(&pthr->dma_sync);
	wake_up(&pthr->dma_wait);
}

static int perf_copy_chunk(struct perf_thread *pthr,
			   void __iomem *dst, void *src, size_t len)
{
	struct dma_async_tx_descriptor *tx;
	struct dmaengine_unmap_data *unmap;
	struct device *dma_dev;
	int try = 0, ret = 0;
	struct perf_peer *peer = pthr->perf->test_peer;
	void __iomem *vbase;
	void __iomem *dst_vaddr;
	dma_addr_t dst_dma_addr;

	if (!use_dma) {
		memcpy_toio(dst, src, len);
		goto ret_check_tsync;
	}

	dma_dev = pthr->dma_chan->device->dev;

	if (!is_dma_copy_aligned(pthr->dma_chan->device, offset_in_page(src),
				 offset_in_page(dst), len))
		return -EIO;

	vbase = peer->outbuf;
	dst_vaddr = dst;
	dst_dma_addr = peer->dma_dst_addr + (dst_vaddr - vbase);

	unmap = dmaengine_get_unmap_data(dma_dev, 1, GFP_NOWAIT);
	if (!unmap)
		return -ENOMEM;

	unmap->len = len;
	unmap->addr[0] = dma_map_page(dma_dev, virt_to_page(src),
		offset_in_page(src), len, DMA_TO_DEVICE);
	if (dma_mapping_error(dma_dev, unmap->addr[0])) {
		ret = -EIO;
		goto err_free_resource;
	}
	unmap->to_cnt = 1;

	do {
		tx = dmaengine_prep_dma_memcpy(pthr->dma_chan, dst_dma_addr,
			unmap->addr[0], len, DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
		if (!tx)
			msleep(DMA_MDELAY);
	} while (!tx && (try++ < DMA_TRIES));

	if (!tx) {
		ret = -EIO;
		goto err_free_resource;
	}

	tx->callback = perf_dma_copy_callback;
	tx->callback_param = pthr;
	dma_set_unmap(tx, unmap);

	ret = dma_submit_error(dmaengine_submit(tx));
	if (ret) {
		dmaengine_unmap_put(unmap);
		goto err_free_resource;
	}

	dmaengine_unmap_put(unmap);

	atomic_inc(&pthr->dma_sync);
	dma_async_issue_pending(pthr->dma_chan);

ret_check_tsync:
	return likely(atomic_read(&pthr->perf->tsync) > 0) ? 0 : -EINTR;

err_free_resource:
	dmaengine_unmap_put(unmap);

	return ret;
}

static bool perf_dma_filter(struct dma_chan *chan, void *data)
{
	struct perf_ctx *perf = data;
	int node;

	node = dev_to_node(&perf->ntb->dev);

	return node == NUMA_NO_NODE || node == dev_to_node(chan->device->dev);
}

static int perf_init_test(struct perf_thread *pthr)
{
	struct perf_ctx *perf = pthr->perf;
	dma_cap_mask_t dma_mask;
	struct perf_peer *peer = pthr->perf->test_peer;

	pthr->src = kmalloc_node(perf->test_peer->outbuf_size, GFP_KERNEL,
				 dev_to_node(&perf->ntb->dev));
	if (!pthr->src)
		return -ENOMEM;

	get_random_bytes(pthr->src, perf->test_peer->outbuf_size);

    dev_info(&perf->ntb->dev, "%s(): dev(%p) out_phys_addr[x%llX] outbuf_size[x%llX] (Ln %d)\n",
        __func__, &perf->ntb->dev, peer->out_phys_addr, peer->outbuf_size, __LINE__); // KW_DB

	if (!use_dma)
		return 0;

	dma_cap_zero(dma_mask);
	dma_cap_set(DMA_MEMCPY, dma_mask);
	pthr->dma_chan = dma_request_channel(dma_mask, perf_dma_filter, perf);
	if (!pthr->dma_chan) {
		dev_err(&perf->ntb->dev, "%d: Failed to get DMA channel\n",
			pthr->tidx);
		goto err_free;
	}


	peer->dma_dst_addr =
		dma_map_resource(pthr->dma_chan->device->dev,
				 peer->out_phys_addr, peer->outbuf_size,
				 DMA_FROM_DEVICE, 0);
	if (dma_mapping_error(pthr->dma_chan->device->dev,
			      peer->dma_dst_addr)) {
		dev_err(pthr->dma_chan->device->dev, "%d: Failed to map DMA addr\n",
			pthr->tidx);
		peer->dma_dst_addr = 0;
		dma_release_channel(pthr->dma_chan);
		goto err_free;
	}
	dev_dbg(pthr->dma_chan->device->dev, "%d: Map MMIO %pa to DMA addr %pad\n",
			pthr->tidx,
			&peer->out_phys_addr,
			&peer->dma_dst_addr);

	atomic_set(&pthr->dma_sync, 0);
	return 0;

err_free:
	atomic_dec(&perf->tsync);
	wake_up(&perf->twait);
	kfree(pthr->src);
	return -ENODEV;
}

static int perf_run_test(struct perf_thread *pthr)
{
	struct perf_peer *peer = pthr->perf->test_peer;
	struct perf_ctx *perf = pthr->perf;
	void __iomem *flt_dst, *bnd_dst;
	u64 total_size, chunk_size;
	void *flt_src;
	int ret = 0;

	total_size = 1ULL << total_order;
	chunk_size = 1ULL << chunk_order;
	chunk_size = min_t(u64, peer->outbuf_size, chunk_size);

	flt_src = pthr->src;
	bnd_dst = peer->outbuf + peer->outbuf_size;
	flt_dst = peer->outbuf;

	pthr->duration = ktime_get();

	/* Copied field is cleared on test launch stage */
	while (pthr->copied < total_size) {
		ret = perf_copy_chunk(pthr, flt_dst, flt_src, chunk_size);
		if (ret) {
			dev_err(&perf->ntb->dev, "%d: Got error %d on test\n",
				pthr->tidx, ret);
			return ret;
		}

		pthr->copied += chunk_size;

		flt_dst += chunk_size;
		flt_src += chunk_size;
		if (flt_dst >= bnd_dst || flt_dst < peer->outbuf) {
			flt_dst = peer->outbuf;
			flt_src = pthr->src;
		}

		/* Give up CPU to give a chance for other threads to use it */
		schedule();
	}

	return 0;
}

static int perf_sync_test(struct perf_thread *pthr)
{
	struct perf_ctx *perf = pthr->perf;

	if (!use_dma)
		goto no_dma_ret;

	wait_event(pthr->dma_wait,
		   (atomic_read(&pthr->dma_sync) == 0 ||
		    atomic_read(&perf->tsync) < 0));

	if (atomic_read(&perf->tsync) < 0)
		return -EINTR;

no_dma_ret:
	pthr->duration = ktime_sub(ktime_get(), pthr->duration);

	dev_dbg(&perf->ntb->dev, "%d: copied %llu bytes\n",
		pthr->tidx, pthr->copied);

	dev_dbg(&perf->ntb->dev, "%d: lasted %llu usecs\n",
		pthr->tidx, ktime_to_us(pthr->duration));

	dev_dbg(&perf->ntb->dev, "%d: %llu MBytes/s\n", pthr->tidx,
		div64_u64(pthr->copied, ktime_to_us(pthr->duration)));

	return 0;
}

static void perf_clear_test(struct perf_thread *pthr)
{
	struct perf_ctx *perf = pthr->perf;

	if (!use_dma)
		goto no_dma_notify;

	/*
	 * If test finished without errors, termination isn't needed.
	 * We call it anyway just to be sure of the transfers completion.
	 */
	(void)dmaengine_terminate_sync(pthr->dma_chan);
	if (pthr->perf->test_peer->dma_dst_addr)
		dma_unmap_resource(pthr->dma_chan->device->dev,
				   pthr->perf->test_peer->dma_dst_addr,
				   pthr->perf->test_peer->outbuf_size,
				   DMA_FROM_DEVICE, 0);

	dma_release_channel(pthr->dma_chan);

no_dma_notify:
	atomic_dec(&perf->tsync);
	wake_up(&perf->twait);
	kfree(pthr->src);
}

static void perf_thread_work(struct work_struct *work)
{
	struct perf_thread *pthr = to_thread_work(work);
	int ret;

	/*
	 * Perform stages in compliance with use_dma flag value.
	 * Test status is changed only if error happened, otherwise
	 * status -ENODATA is kept while test is on-fly. Results
	 * synchronization is performed only if test fininshed
	 * without an error or interruption.
	 */
    printk("%s(): Calling perf_init_test() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	ret = perf_init_test(pthr);
	if (ret) {
		pthr->status = ret;
		return;
	}

	printk("%s(): Calling perf_run_test() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
    ret = perf_run_test(pthr);
	if (ret) {
		pthr->status = ret;
		goto err_clear_test;
	}

	printk("%s(): Calling perf_sync_test() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	pthr->status = perf_sync_test(pthr);

err_clear_test:
	printk("%s(): Calling perf_clear_test() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	perf_clear_test(pthr);
}

static int perf_set_tcnt(struct perf_ctx *perf, u8 tcnt)
{
	if (tcnt == 0 || tcnt > MAX_THREADS_CNT)
		return -EINVAL;

	if (test_and_set_bit_lock(0, &perf->busy_flag))
		return -EBUSY;

	perf->tcnt = tcnt;

	clear_bit_unlock(0, &perf->busy_flag);

	return 0;
}

static void perf_terminate_test(struct perf_ctx *perf)
{
	int tidx;

	atomic_set(&perf->tsync, -1);
	wake_up(&perf->twait);

	for (tidx = 0; tidx < MAX_THREADS_CNT; tidx++) {
		wake_up(&perf->threads[tidx].dma_wait);
		cancel_work_sync(&perf->threads[tidx].work);
	}
}

static int perf_submit_test(struct perf_peer *peer)
{
	struct perf_ctx *perf = peer->perf;
	struct perf_thread *pthr;
	int tidx, ret;

    dev_info(&perf->ntb->dev, "%s(): Calling wait_for_completion_interruptible() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	ret = wait_for_completion_interruptible(&peer->init_comp);
    dev_info(&perf->ntb->dev, "%s(): Returned from wait_for_completion_interruptible() ret[%d] (Ln %d)\n",
        __func__, ret, __LINE__); // KW_DB
	if (ret < 0)
		return ret;

	if (test_and_set_bit_lock(0, &perf->busy_flag))
		return -EBUSY;

	perf->test_peer = peer;
	atomic_set(&perf->tsync, perf->tcnt);

	for (tidx = 0; tidx < MAX_THREADS_CNT; tidx++) {
		pthr = &perf->threads[tidx];

		pthr->status = -ENODATA;
		pthr->copied = 0;
		pthr->duration = ktime_set(0, 0);
		if (tidx < perf->tcnt)
			(void)queue_work(perf_wq, &pthr->work);
	}

	ret = wait_event_interruptible(perf->twait,
				       atomic_read(&perf->tsync) <= 0);
	if (ret == -ERESTARTSYS) {
		perf_terminate_test(perf);
		ret = -EINTR;
	}

	clear_bit_unlock(0, &perf->busy_flag);

	return ret;
}

static int perf_read_stats(struct perf_ctx *perf, char *buf,
			   size_t size, ssize_t *pos)
{
	struct perf_thread *pthr;
	int tidx;

	if (test_and_set_bit_lock(0, &perf->busy_flag))
		return -EBUSY;

	(*pos) += scnprintf(buf + *pos, size - *pos,
		"    Peer %d test statistics:\n", perf->test_peer->pidx);

	for (tidx = 0; tidx < MAX_THREADS_CNT; tidx++) {
		pthr = &perf->threads[tidx];

		if (pthr->status == -ENODATA)
			continue;

		if (pthr->status) {
			(*pos) += scnprintf(buf + *pos, size - *pos,
				"%d: error status %d\n", tidx, pthr->status);
			continue;
		}

		(*pos) += scnprintf(buf + *pos, size - *pos,
			"%d: copied %llu bytes in %llu usecs, %llu MBytes/s\n",
			tidx, pthr->copied, ktime_to_us(pthr->duration),
			div64_u64(pthr->copied, ktime_to_us(pthr->duration)));
	}

	clear_bit_unlock(0, &perf->busy_flag);

	return 0;
}

static void perf_init_threads(struct perf_ctx *perf)
{
	struct perf_thread *pthr;
	int tidx;

	perf->tcnt = DEF_THREADS_CNT;
	perf->test_peer = &perf->peers[0];
	init_waitqueue_head(&perf->twait);

	for (tidx = 0; tidx < MAX_THREADS_CNT; tidx++) {
		pthr = &perf->threads[tidx];

		pthr->perf = perf;
		pthr->tidx = tidx;
		pthr->status = -ENODATA;
		init_waitqueue_head(&pthr->dma_wait);
		INIT_WORK(&pthr->work, perf_thread_work);
	}
}

static void perf_clear_threads(struct perf_ctx *perf)
{
	perf_terminate_test(perf);
}

/*==============================================================================
 *                               DebugFS nodes
 *==============================================================================
 */

static ssize_t perf_dbgfs_read_info(struct file *filep, char __user *ubuf,
				    size_t size, loff_t *offp)
{
	struct perf_ctx *perf = filep->private_data;
	struct perf_peer *peer;
	size_t buf_size;
	ssize_t pos = 0;
	int ret, pidx;
	char *buf;

	buf_size = min_t(size_t, size, 0x1000U);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	pos += scnprintf(buf + pos, buf_size - pos,
		"    Performance measuring tool info:\n\n");

	pos += scnprintf(buf + pos, buf_size - pos,
		"Local port %d, Global index %d\n", ntb_port_number(perf->ntb),
		perf->gidx);
	pos += scnprintf(buf + pos, buf_size - pos, "Test status: ");
	if (test_bit(0, &perf->busy_flag)) {
		pos += scnprintf(buf + pos, buf_size - pos,
			"on-fly with port %d (%d)\n",
			ntb_peer_port_number(perf->ntb, perf->test_peer->pidx),
			perf->test_peer->pidx);
	} else {
		pos += scnprintf(buf + pos, buf_size - pos, "idle\n");
	}

	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		peer = &perf->peers[pidx];

		pos += scnprintf(buf + pos, buf_size - pos,
			"Port %d (%d), Global index %d:\n",
			ntb_peer_port_number(perf->ntb, peer->pidx), peer->pidx,
			peer->gidx);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tLink status: %s\n",
			test_bit(PERF_STS_LNKUP, &peer->sts) ? "up" : "down");

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tOut buffer addr 0x%pK\n", peer->outbuf);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tOut buff phys addr %pa[p]\n", &peer->out_phys_addr);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tOut buffer size %pa\n", &peer->outbuf_size);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tOut buffer xlat 0x%016llx[p]\n", peer->outbuf_xlat);

		if (!peer->inbuf) {
			pos += scnprintf(buf + pos, buf_size - pos,
				"\tIn buffer addr: unallocated\n");
			continue;
		}

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer addr 0x%pK\n", peer->inbuf);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer size %pa\n", &peer->inbuf_size);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer xlat %pad[p]\n", &peer->inbuf_xlat);
	}

	ret = simple_read_from_buffer(ubuf, size, offp, buf, pos);
	kfree(buf);

	return ret;
}

static const struct file_operations perf_dbgfs_info = {
	.open = simple_open,
	.read = perf_dbgfs_read_info
};

static ssize_t perf_dbgfs_read_run(struct file *filep, char __user *ubuf,
				   size_t size, loff_t *offp)
{
	struct perf_ctx *perf = filep->private_data;
	ssize_t ret, pos = 0;
	char *buf;

	buf = kmalloc(PERF_BUF_LEN, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = perf_read_stats(perf, buf, PERF_BUF_LEN, &pos);
	if (ret)
		goto err_free;

	ret = simple_read_from_buffer(ubuf, size, offp, buf, pos);
err_free:
	kfree(buf);

	return ret;
}

static ssize_t perf_dbgfs_write_run(struct file *filep, const char __user *ubuf,
				    size_t size, loff_t *offp)
{
	struct perf_ctx *perf = filep->private_data;
	struct perf_peer *peer;
	int pidx, ret;

	ret = kstrtoint_from_user(ubuf, size, 0, &pidx);
	if (ret)
		return ret;

    dev_info(&perf->ntb->dev, "%s(): pcnt[%d] (Ln %d)\n",
        __func__, perf->pcnt, __LINE__); // KW_DB
	if (pidx < 0 || pidx >= perf->pcnt)
		return -EINVAL;

	peer = &perf->peers[pidx];

    dev_info(&perf->ntb->dev, "%s(): pidx[%d] peer(%p) (Ln %d)\n",
        __func__, pidx, peer, __LINE__); // KW_DB

	ret = perf_submit_test(peer);
	if (ret)
		return ret;

	return size;
}

static const struct file_operations perf_dbgfs_run = {
	.open = simple_open,
	.read = perf_dbgfs_read_run,
	.write = perf_dbgfs_write_run
};

static ssize_t perf_dbgfs_read_tcnt(struct file *filep, char __user *ubuf,
				    size_t size, loff_t *offp)
{
	struct perf_ctx *perf = filep->private_data;
	char buf[8];
	ssize_t pos;

	pos = scnprintf(buf, sizeof(buf), "%hhu\n", perf->tcnt);

	return simple_read_from_buffer(ubuf, size, offp, buf, pos);
}

static ssize_t perf_dbgfs_write_tcnt(struct file *filep,
				     const char __user *ubuf,
				     size_t size, loff_t *offp)
{
	struct perf_ctx *perf = filep->private_data;
	int ret;
	u8 val;

	ret = kstrtou8_from_user(ubuf, size, 0, &val);
	if (ret)
		return ret;

	ret = perf_set_tcnt(perf, val);
	if (ret)
		return ret;

	return size;
}

static const struct file_operations perf_dbgfs_tcnt = {
	.open = simple_open,
	.read = perf_dbgfs_read_tcnt,
	.write = perf_dbgfs_write_tcnt
};

// KW_DB->
static ssize_t perf_dbgfs_read_inbuf(struct file *filep, char __user *ubuf,
				                     size_t size, loff_t *offp)
{
	struct perf_ctx    *perf = filep->private_data;
	struct perf_peer   *peer;
	size_t              buf_size;
	ssize_t             pos = 0;
	int                 ret, pidx;
	char               *buf;
    unsigned int        offset = 0;

	// ret = kstrtouint_from_user(ubuf, size, 0, &offset);
	// if (ret)
		// return ret;

	buf_size = min_t(size_t, size, 0x1000U);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	// pos += scnprintf(buf + pos, buf_size - pos,
		// "    Performance measuring tool info:\n\n");

	// pos += scnprintf(buf + pos, buf_size - pos,
		// "Local port %d, Global index %d\n", ntb_port_number(perf->ntb),
		// perf->gidx);
	// pos += scnprintf(buf + pos, buf_size - pos, "Test status: ");
	// if (test_bit(0, &perf->busy_flag)) {
		// pos += scnprintf(buf + pos, buf_size - pos,
			// "on-fly with port %d (%d)\n",
			// ntb_peer_port_number(perf->ntb, perf->test_peer->pidx),
			// perf->test_peer->pidx);
	// } else {
		// pos += scnprintf(buf + pos, buf_size - pos, "idle\n");
	// }

	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		peer = &perf->peers[pidx];

		// pos += scnprintf(buf + pos, buf_size - pos,
			// "Port %d (%d), Global index %d:\n",
			// ntb_peer_port_number(perf->ntb, peer->pidx), peer->pidx,
			// peer->gidx);

		// pos += scnprintf(buf + pos, buf_size - pos,
			// "\tLink status: %s\n",
			// test_bit(PERF_STS_LNKUP, &peer->sts) ? "up" : "down");

		// pos += scnprintf(buf + pos, buf_size - pos,
			// "\tOut buffer addr 0x%pK\n", peer->outbuf);

		// pos += scnprintf(buf + pos, buf_size - pos,
			// "\tOut buff phys addr %pa[p]\n", &peer->out_phys_addr);

		// pos += scnprintf(buf + pos, buf_size - pos,
			// "\tOut buffer size %pa\n", &peer->outbuf_size);

		// pos += scnprintf(buf + pos, buf_size - pos,
			// "\tOut buffer xlat 0x%016llx[p]\n", peer->outbuf_xlat);

		if (!peer->inbuf) {
			pos += scnprintf(buf + pos, buf_size - pos,
				"\tIn buffer addr: unallocated\n");
			continue;
		}

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer addr 0x%pK\n", peer->inbuf);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer size %pa\n", &peer->inbuf_size);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer xlat %pad[p]\n", &peer->inbuf_xlat);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer (offset x%X data): 0x%X\n", offset, *(u32 *) (peer->inbuf + offset));
	}

	ret = simple_read_from_buffer(ubuf, size, offp, buf, pos);
	kfree(buf);

	return ret;
}

static const struct file_operations perf_dbgfs_inbuf = {
	.open = simple_open,
	.read = perf_dbgfs_read_inbuf
};

static ssize_t perf_dbgfs_read_pciCfgBuf(struct file *filep, char __user *ubuf,
				                         size_t size, loff_t *offp)
{
	struct perf_ctx    *perf = filep->private_data;
	struct perf_peer   *peer;
	size_t              buf_size;
	ssize_t             pos = 0;
	int                 ret, pidx;
	char               *buf;
    unsigned int        offset = 0;

	// ret = kstrtouint_from_user(ubuf, size, 0, &offset);
	// if (ret)
		// return ret;

	buf_size = min_t(size_t, size, 0x1000U);

	buf = kmalloc(buf_size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		peer = &perf->peers[pidx];

		if (!peer->pciCfgBuf) {
			pos += scnprintf(buf + pos, buf_size - pos,
				"\tIn buffer addr: unallocated\n");
			continue;
		}

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer addr 0x%pK\n", peer->pciCfgBuf);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer size %pa\n", &peer->pciCfgBuf_size);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer xlat %pad[p]\n", &peer->pciCfgBuf_xlat);

		pos += scnprintf(buf + pos, buf_size - pos,
			"\tIn buffer (offset x%X data): 0x%X\n", offset, *(u32 *) (peer->pciCfgBuf + offset));
	}

	ret = simple_read_from_buffer(ubuf, size, offp, buf, pos);
	kfree(buf);

	return ret;
}

static const struct file_operations perf_dbgfs_pciCfgBuf = {
	.open = simple_open,
	.read = perf_dbgfs_read_pciCfgBuf
};

static ssize_t perf_dbgfs_write_stask(struct file *filep,
				                      const char __user *ubuf,
				                      size_t size, loff_t *offp)
{
	struct perf_ctx    *perf = filep->private_data;
    struct perf_peer   *peer= &perf->peers[0];
	int                 ret;
	u8                  val;

	ret = kstrtou8_from_user(ubuf, size, 0, &val);
	if (ret)
		return ret;

    ret = size;
    switch(val)
    {
        case 1:
            dev_info(&perf->ntb->dev, "%s(): DMA map... (Ln %d)\n",
                __func__, __LINE__); // KW_DB
            dev_info(&perf->ntb->dev, "%s(): dev(%p) out_phys_addr[x%llX] outbuf_size[x%llX] (Ln %d)\n",
                __func__, &perf->ntb->dev, peer->out_phys_addr, peer->outbuf_size, __LINE__); // KW_DB

            perf->dma_handle = dma_map_resource(&perf->ntb->dev,
                                                 peer->out_phys_addr,
                                                 // 0xB0000000,
                                                 peer->outbuf_size,
                                                 // DMA_TO_DEVICE,
                                                 DMA_BIDIRECTIONAL,
                                                 0);
            if (dma_mapping_error(&perf->ntb->dev,
			      perf->dma_handle))
            {
                dev_info(&perf->ntb->dev, "%s(): DMA map failed! (Ln %d)\n",
                    __func__, __LINE__); // KW_DB
                ret = -ENOMEM;
            }
            else
                dev_info(&perf->ntb->dev, "%s(): dma_handle[x%llX] (Ln %d)\n",
                    __func__, (u64) perf->dma_handle, __LINE__); // KW_DB
            break;
        case 2:
            dev_info(&perf->ntb->dev, "%s(): DMA unmap... (Ln %d)\n",
                __func__, __LINE__); // KW_DB
            dma_unmap_resource(&perf->ntb->dev,
                               perf->dma_handle,
                               peer->outbuf_size,
		                       DMA_TO_DEVICE,
                               0);
            break;
        case 3: // Allocated memory if not already done so.
            if (perf->pBuf_virt == NULL)
                perf->pBuf_virt = (void *) __get_free_pages(GFP_KERNEL, get_order(peer->outbuf_size));
            dev_info(&perf->ntb->dev, "%s(): pBuf_virt[x%llX] pBuf_phys[x%llX] (Ln %d)\n",
                __func__, (u64) perf->pBuf_virt, (u64) virt_to_phys(perf->pBuf_virt),__LINE__);
            break;
        case 4: // Deallocate memory.
            free_pages((u64) perf->pBuf_virt, get_order(peer->outbuf_size));
            perf->pBuf_virt = NULL;
            dev_info(&perf->ntb->dev, "%s(): pBuf_virtb deallocated (Ln %d)\n",
                __func__, __LINE__); // KW_DB
            break;
        case 5: // Dump memory content.
            dev_info(&perf->ntb->dev, "%s(): pBuf_virt[x%llX] pBuf_phys[x%llX] Content[x%X] (Ln %d)\n",
                __func__, (u64) perf->pBuf_virt,
                (u64) virt_to_phys(perf->pBuf_virt),
                * (u32 *) perf->pBuf_virt, __LINE__);
            break;
        case 10: // Send Custom command. KW_DB
            perf->cmd_send(peer, PERF_CMD_DEV_ADD, 0x12345678);
            break;
        default:
            dev_info(&perf->ntb->dev, "%s(): Unknown task[%d] (Ln %d)\n",
                __func__, val, __LINE__); // KW_DB
                ret = -EINVAL;
    }


	return ret;
}

static const struct file_operations perf_dbgfs_stask = {
	.open = simple_open,
	.write = perf_dbgfs_write_stask
};
// KW_DB<-

static void perf_setup_dbgfs(struct perf_ctx *perf)
{
	struct pci_dev *pdev = perf->ntb->pdev;

	perf->dbgfs_dir = debugfs_create_dir(pci_name(pdev), perf_dbgfs_topdir);
	if (!perf->dbgfs_dir) {
		dev_warn(&perf->ntb->dev, "DebugFS unsupported\n");
		return;
	}

	debugfs_create_file("info", 0600, perf->dbgfs_dir, perf,
			    &perf_dbgfs_info);

	debugfs_create_file("run", 0600, perf->dbgfs_dir, perf,
			    &perf_dbgfs_run);

	debugfs_create_file("threads_count", 0600, perf->dbgfs_dir, perf,
			    &perf_dbgfs_tcnt);

	debugfs_create_file("inbuf", 0600, perf->dbgfs_dir, perf,
			    &perf_dbgfs_inbuf);  // KW_DB

    debugfs_create_file("pciCfgBuf", 0600, perf->dbgfs_dir, perf,
			    &perf_dbgfs_pciCfgBuf);  // KW_DB

	debugfs_create_file("stask", 0600, perf->dbgfs_dir, perf,
			    &perf_dbgfs_stask);  // KW_DB

	/* They are made read-only for test exec safety and integrity */
	debugfs_create_u8("chunk_order", 0500, perf->dbgfs_dir, &chunk_order);

	debugfs_create_u8("total_order", 0500, perf->dbgfs_dir, &total_order);

	debugfs_create_bool("use_dma", 0500, perf->dbgfs_dir, &use_dma);
}

static void perf_clear_dbgfs(struct perf_ctx *perf)
{
	debugfs_remove_recursive(perf->dbgfs_dir);
}

/*==============================================================================
 *                        Basic driver initialization
 *==============================================================================
 */

static struct perf_ctx *perf_create_data(struct ntb_dev *ntb)
{
	struct perf_ctx *perf;

	perf = devm_kzalloc(&ntb->dev, sizeof(*perf), GFP_KERNEL);
	if (!perf)
		return ERR_PTR(-ENOMEM);

	perf->pcnt = ntb_peer_port_count(ntb);
    ///perf->pcnt =2;  // KW_DB - exp
    dev_info(&ntb->dev, "%s(): pcnt[%d] ntb->topo[%d] (Ln %d)\n",
        __func__, perf->pcnt, ntb->topo ,__LINE__); // KW_DB
    ///if (!ntb->ops->peer_port_count)
    ///    dev_info(&ntb->dev, "%s(): No peer_port_count() (Ln %d)\n",
    ///    __func__,__LINE__); // KW_DB
    ///else
    ///    dev_info(&ntb->dev, "%s(): Yes peer_port_count() (Ln %d)\n",
    ///    __func__,__LINE__); // KW_DB
	perf->peers = devm_kcalloc(&ntb->dev, perf->pcnt, sizeof(*perf->peers),
				  GFP_KERNEL);
	if (!perf->peers)
		return ERR_PTR(-ENOMEM);

	perf->ntb = ntb;

	return perf;
}

static int perf_setup_peer_mw(struct perf_peer *peer)
{
	struct perf_ctx *perf = peer->perf;
	phys_addr_t phys_addr;
	int ret;

	/* Get outbound MW parameters and map it */
	ret = ntb_peer_mw_get_addr(perf->ntb, perf->gidx, &phys_addr,
				   &peer->outbuf_size);
	if (ret)
		return ret;

	peer->outbuf = devm_ioremap_wc(&perf->ntb->dev, phys_addr,
        peer->outbuf_size);/// KW_DB
    dev_info(&perf->ntb->dev, "%s(): outbuf(%p) phys_addr[x%llX] (Ln %d)\n",
        __func__, peer->outbuf, (u64) phys_addr, __LINE__); // KW_DB
    dev_info(&perf->ntb->dev, "%s(): Converted phys[x%llX] (Ln %d)\n",
        __func__, (u64) virt_to_phys(peer->outbuf), __LINE__); // KW_DB
	if (!peer->outbuf)
		return -ENOMEM;

	peer->out_phys_addr = phys_addr;

	if (max_mw_size && peer->outbuf_size > max_mw_size) {
		peer->outbuf_size = max_mw_size;
		dev_warn(&peer->perf->ntb->dev,
			"Peer %d outbuf reduced to %pa\n", peer->pidx,
			&peer->outbuf_size);
	}

	return 0;
}

static int perf_init_peers(struct perf_ctx *perf)
{
	struct perf_peer *peer;
	int pidx, lport, ret;

	lport = ntb_port_number(perf->ntb);
	perf->gidx = -1;
	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		peer = &perf->peers[pidx];

		peer->perf = perf;
		peer->pidx = pidx;
		if (lport < ntb_peer_port_number(perf->ntb, pidx)) {
			if (perf->gidx == -1)
				perf->gidx = pidx;
			peer->gidx = pidx + 1;
		} else {
			peer->gidx = pidx;
		}
		INIT_WORK(&peer->service, perf_service_work);
		init_completion(&peer->init_comp);
	}
	if (perf->gidx == -1)
		perf->gidx = pidx;

	/*
	 * Hardware with only two ports may not have unique port
	 * numbers. In this case, the gidxs should all be zero.
	 */
	if (perf->pcnt == 1 &&  ntb_port_number(perf->ntb) == 0 &&
	    ntb_peer_port_number(perf->ntb, 0) == 0) {
		perf->gidx = 0;
		perf->peers[0].gidx = 0;
	}

	for (pidx = 0; pidx < perf->pcnt; pidx++) {
		ret = perf_setup_peer_mw(&perf->peers[pidx]);
		if (ret)
			return ret;
	}

	dev_dbg(&perf->ntb->dev, "Global port index %d\n", perf->gidx);

	return 0;
}

#if (SK_HYNIX == 1)
// extern int lending_user_init(struct perf_ctx *perf,struct proc_dir_entry *proc_sio_root); //
// extern int lending_user_exit(struct perf_ctx *perf); //
extern int borrow_user_init(struct perf_ctx *perf, struct proc_dir_entry *proc_skio_root);
extern int borrow_user_exit(struct perf_ctx *perf);
///int user_init(struct perf_ctx *perf);   // KW_DB_20240815 - 6.8.0-40-generic - error: no previous prototype for ‘user_exit’ [-Werror=missing-prototypes]
///int user_exit(struct perf_ctx *perf);   // KW_DB_20240815 - 6.8.0-40-generic - error: no previous prototype for ‘user_exit’ [-Werror=missing-prototypes]
/**
 *
 */
int user_init(struct perf_ctx *perf)
{
    int ret;

    pr_info("%s(): Here\n", __func__);
    proc_skio_root = proc_mkdir(PROC_SKIO, NULL );
    if(!proc_skio_root) {
        return -1;
    }
    perf->proc_skio_root = proc_skio_root;

    ret = lending_user_init(perf, proc_skio_root);
    if(ret) {
        proc_remove(proc_skio_root);
        return ret;
    }

    ret = borrow_user_init(perf, proc_skio_root);
    if(ret) {
        lending_user_exit(perf);
        proc_remove(proc_skio_root);
        return ret;
    }

    return 0;
}

/**
 *
 */
int user_exit(struct perf_ctx *perf)
{
    pr_info("%s(): Entered (Ln %d)\n", __func__, __LINE__);    // KW_DB
    lending_user_exit(perf);
    borrow_user_exit(perf);
    // proc_remove(proc_skio_root);
    ///remove_proc_entry(PROC_SKIO,NULL);
    proc_remove(proc_skio_root);
    pr_info("%s(): Leaving (Ln %d)\n", __func__, __LINE__);    // KW_DB

    return 0;
}
#endif

static int perf_probe(struct ntb_client *client, struct ntb_dev *ntb)
{
	struct perf_ctx *perf;
	int ret;

	perf = perf_create_data(ntb);
	if (IS_ERR(perf))
		return PTR_ERR(perf);

    printk("%s(): mw_count[%d]. (Ln %d)\n",
        __func__, ntb_mw_count(perf->ntb,0), __LINE__); // KW_DB

	ret = perf_init_peers(perf);
	if (ret)
		return ret;

	perf_init_threads(perf);

	ret = perf_init_service(perf);
	if (ret)
		return ret;

	ret = perf_enable_service(perf);
	if (ret)
		return ret;

	perf_setup_dbgfs(perf);

#if (SK_HYNIX == 1)
    user_init(perf);
#endif

	return 0;
}

static void perf_remove(struct ntb_client *client, struct ntb_dev *ntb)
{
	struct perf_ctx *perf = ntb->ctx;

#if (SK_HYNIX == 1)
    user_exit(perf);
#endif

	perf_clear_dbgfs(perf);

	perf_disable_service(perf);

	perf_clear_threads(perf);
}

static struct ntb_client perf_client = {
	.ops = {
		.probe = perf_probe,
		.remove = perf_remove
	}
};

static int __init perf_init(void)
{
	int ret;

	if (chunk_order > MAX_CHUNK_ORDER) {
		chunk_order = MAX_CHUNK_ORDER;
		pr_info("Chunk order reduced to %hhu\n", chunk_order);
	}

	if (total_order < chunk_order) {
		total_order = chunk_order;
		pr_info("Total data order reduced to %hhu\n", total_order);
	}

	perf_wq = alloc_workqueue("perf_wq", WQ_UNBOUND | WQ_SYSFS, 0);
	if (!perf_wq)
		return -ENOMEM;

	if (debugfs_initialized())
		perf_dbgfs_topdir = debugfs_create_dir(KBUILD_MODNAME, NULL);

	ret = ntb_register_client(&perf_client);
	if (ret) {
		debugfs_remove_recursive(perf_dbgfs_topdir);
		destroy_workqueue(perf_wq);
	}

	return ret;
}
module_init(perf_init);

static void __exit perf_exit(void)
{
	ntb_unregister_client(&perf_client);
	debugfs_remove_recursive(perf_dbgfs_topdir);
	destroy_workqueue(perf_wq);
}
module_exit(perf_exit);
