/*
 * Microsemi Switchtec(tm) PCIe Management Driver
 * Copyright (c) 2017, Microsemi Corporation
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/switchtec.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/ntb.h>
#include <linux/version.h>  // KW_DB

#include "version.h"
MODULE_DESCRIPTION("Microsemi Switchtec(tm) NTB Driver");
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Microsemi Corporation");

static ulong max_mw_size = SZ_2M;
module_param(max_mw_size, ulong, 0644);
MODULE_PARM_DESC(max_mw_size,
	"Max memory window size reported to the upper layer");

static bool use_lut_mws;
module_param(use_lut_mws, bool, 0644);
MODULE_PARM_DESC(use_lut_mws,
		 "Enable the use of the LUT based memory windows");

#ifndef ioread64
#ifdef readq
#define ioread64 readq
#else
#define ioread64 _ioread64
static inline u64 _ioread64(void __iomem *mmio)
{
	u64 low, high;

	low = ioread32(mmio);
	high = ioread32(mmio + sizeof(u32));
	return low | (high << 32);
}
#endif
#endif

#ifndef iowrite64
#ifdef writeq
#define iowrite64 writeq
#else
#define iowrite64 _iowrite64
static inline void _iowrite64(u64 val, void __iomem *mmio)
{
	iowrite32(val, mmio);
	iowrite32(val >> 32, mmio + sizeof(u32));
}
#endif
#endif

#define SWITCHTEC_NTB_MAGIC 0x45CC0001
#define MAX_MWS     128  // TODO:  May need to adjjust,

struct shared_mw {
	u32 magic;
	u32 link_sta;
	u32 partition_id;
	u64 mw_sizes[MAX_MWS];
	//u32 spad[128];    // KW_DB - Original setting
    u32 spad[2048];     // KW_DB - Allows 1 entry for CMD and the rest data.
};

#define MAX_DIRECT_MW ARRAY_SIZE(((struct ntb_ctrl_regs *)(0))->bar_entry)
// #define LUT_SIZE SZ_64K // KW_DB
#define LUT_SIZE SZ_32K    // KW_DB

struct switchtec_ntb {
	struct ntb_dev ntb;
	struct switchtec_dev *stdev;

	int self_partition;
	int peer_partition;

	int doorbell_irq;
	int message_irq;

	struct ntb_info_regs __iomem *mmio_ntb;
	struct ntb_ctrl_regs __iomem *mmio_ctrl;
	struct ntb_dbmsg_regs __iomem *mmio_dbmsg;
	struct ntb_ctrl_regs __iomem *mmio_self_ctrl;
	struct ntb_ctrl_regs __iomem *mmio_peer_ctrl;
	struct ntb_ctrl_regs __iomem *mmio_xlink_peer_ctrl;
	struct ntb_dbmsg_regs __iomem *mmio_self_dbmsg;
	struct ntb_dbmsg_regs __iomem *mmio_peer_dbmsg;

	void __iomem *mmio_xlink_dbmsg_win;
	void __iomem *mmio_xlink_ctrl_win;

	struct shared_mw *self_shared;
	struct shared_mw __iomem *peer_shared;
    
	dma_addr_t self_shared_dma;

#if (SK_HYNIX == 1)
    void           *sp_mw_01_shared;
    dma_addr_t      sp_mw_01_shared_dma;
    
    void __iomem   *peer_sp_mw_01_shared;
#endif

	u64 db_mask;
	u64 db_valid_mask;
	int db_shift;
	int db_peer_shift;

	/* synchronize rmw access of db_mask and hw reg */
	spinlock_t db_mask_lock;

	int nr_direct_mw;
	int nr_lut_mw;
	int nr_rsvd_luts;
	int direct_mw_to_bar[MAX_DIRECT_MW];

	int peer_nr_direct_mw;
	int peer_nr_lut_mw;
	int peer_direct_mw_to_bar[MAX_DIRECT_MW];

	bool link_is_up;
	enum ntb_speed link_speed;
	enum ntb_width link_width;
	struct work_struct check_link_status_work;
	bool link_force_down;
};

static struct switchtec_ntb *ntb_sndev(struct ntb_dev *ntb)
{
	return container_of(ntb, struct switchtec_ntb, ntb);
}

static int switchtec_ntb_part_op_no_retry(struct switchtec_ntb *sndev,
					  struct ntb_ctrl_regs __iomem *ctl,
					  u32 op, int wait_status)
{
	static const char * const op_text[] = {
		[NTB_CTRL_PART_OP_LOCK] = "lock",
		[NTB_CTRL_PART_OP_CFG] = "configure",
		[NTB_CTRL_PART_OP_RESET] = "reset",
	};

	int i;
	u32 ps;
	int status;
	int part_id, locked_part_id;
	int xlink_peer = ctl == sndev->mmio_xlink_peer_ctrl ? 1 : 0;

	ps = ioread32(&ctl->partition_status);

	locked_part_id = (ps & 0xFF0000) >> 16;
	part_id = (ps & 0xFF000000) >> 24;

	ps &= 0xFFFF;
    
    dev_info(&sndev->stdev->dev, "%s(): ps[0x%X] (Ln %d)\n", __func__, ps, __LINE__);  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): part_id[%d] locked_part_id[%d] xlink_peer[%d] (Ln %d)\n", 
        __func__, part_id, locked_part_id, xlink_peer, __LINE__);  // KW_DB

    // KW_DB_20250716->
    //xlink_peer = 2;
    //dev_info(&sndev->stdev->dev, "%s(): Force xlink_peer=%d (Ln %d)\n", __func__, xlink_peer, __LINE__);  // KW_DB
    // KW_DB_20250716<-
    
	if (ps != NTB_CTRL_PART_STATUS_NORMAL &&
	    ps != NTB_CTRL_PART_STATUS_LOCKED)
    {  // KW_DB
        dev_info(&sndev->stdev->dev, "%s(): -EAGAIN (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -EAGAIN;
    }  // KW_DB

	if (ps == NTB_CTRL_PART_STATUS_LOCKED)
		if ((xlink_peer && (locked_part_id != part_id)) ||
		    (!xlink_peer && (locked_part_id != sndev->self_partition)))
        {  // KW_DB
            dev_info(&sndev->stdev->dev, "%s(): -EAGAIN (Ln %d)\n", __func__, __LINE__);  // KW_DB
			return -EAGAIN;
        }  // KW_DB

	switch (op) {
	case NTB_CTRL_PART_OP_LOCK:
		status = NTB_CTRL_PART_STATUS_LOCKING;
		break;
	case NTB_CTRL_PART_OP_CFG:
		status = NTB_CTRL_PART_STATUS_CONFIGURING;
		break;
	case NTB_CTRL_PART_OP_RESET:
		status = NTB_CTRL_PART_STATUS_RESETTING;
		break;
	default:
        dev_info(&sndev->stdev->dev, "%s(): -EINVAL (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -EINVAL;
	}

	iowrite32(op, &ctl->partition_op);

	for (i = 0; i < 1000; i++) {
        // KW_DB->
        #if 1   // Trying to avoid "Voluntary context switch within RCU read-side critical section!" warning.
        mdelay(50) ;
        #else
        // KW_DB<-
		if (msleep_interruptible(50) != 0) {  
			iowrite32(NTB_CTRL_PART_OP_RESET, &ctl->partition_op);
            dev_info(&sndev->stdev->dev, "%s(): -EINTR (Ln %d)\n", __func__, __LINE__);  // KW_DB
			return -EINTR;
		}
        #endif  // KW_DB

		ps = ioread32(&ctl->partition_status);

		locked_part_id = (ps & 0xFF0000) >> 16;
		part_id = (ps & 0xFF000000) >> 24;

		ps &= 0xFFFF;

		if (ps != status)
			break;
	}

	if (ps == NTB_CTRL_PART_STATUS_LOCKED) {
		if ((xlink_peer && (locked_part_id != part_id)) ||
		    (!xlink_peer &&
		     (locked_part_id != sndev->self_partition)))
        {  // KW_DB
            dev_info(&sndev->stdev->dev, "%s(): -EAGAIN (Ln %d)\n", __func__, __LINE__);  // KW_DB
			return -EAGAIN;
        }  // KW_DB
	}

	if (ps == wait_status)
    ///{  // KW_DB
        ///dev_info(&sndev->stdev->dev, "%s(): Leaving.(Ln %d)\n", __func__, __LINE__);  // KW_DB
		return 0;
    ///}  // KW_DB

	if (ps == status) {
		dev_err(&sndev->stdev->dev,
			"Timed out while performing %s (%d). (%08x)\n",
			op_text[op], op,
			ioread32(&ctl->partition_status));

        dev_info(&sndev->stdev->dev, "%s(): -ETIMEDOUT (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -ETIMEDOUT;
	}

	return -EIO;
}

static int switchtec_ntb_part_op(struct switchtec_ntb *sndev,
				 struct ntb_ctrl_regs __iomem *ctl,
				 u32 op, int wait_status)
{
	int rc;
	int i = 0;

    ///dev_info(&sndev->stdev->dev, "%s(): Entered (Ln %d)\n", __func__, __LINE__);  // KW_DB
	while (i++ < 10) {
		rc = switchtec_ntb_part_op_no_retry(sndev, ctl, op,
						    wait_status);
		if (rc == -EAGAIN) {
			if (msleep_interruptible(30) != 0)
            {
                dev_info(&sndev->stdev->dev, "%s(): EINTR (Ln %d)\n", __func__, __LINE__);  // KW_DB
				return -EINTR;
            }
			continue;
		}

		break;
	}

    ///dev_info(&sndev->stdev->dev, "%s(): Leaving (Ln %d)\n", __func__, __LINE__);  // KW_DB
	return rc;
}

static int switchtec_ntb_send_msg(struct switchtec_ntb *sndev, int idx,
				  u32 val)
{
	if (idx < 0 || idx >= ARRAY_SIZE(sndev->mmio_peer_dbmsg->omsg))
		return -EINVAL;

	iowrite32(val, &sndev->mmio_peer_dbmsg->omsg[idx].msg);

	return 0;
}

static int switchtec_ntb_mw_count(struct ntb_dev *ntb, int pidx)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	int nr_direct_mw = sndev->peer_nr_direct_mw;
	int nr_lut_mw = sndev->peer_nr_lut_mw - sndev->nr_rsvd_luts;

    ///dev_info(&sndev->stdev->dev, "%s(): nr_lut_mw[%d] (Ln %d)\n",
    ///    __func__, sndev->nr_lut_mw, __LINE__); // KW_DB
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	if (!use_lut_mws)
		nr_lut_mw = 0;

    ///dev_info(&sndev->stdev->dev, "%s(): nr_direct_mw[%d] nr_lut_mw[%d] (Ln %d)\n",
    ///    __func__, sndev->nr_direct_mw, nr_lut_mw, __LINE__); // KW_DB
        
	return nr_direct_mw + nr_lut_mw;
}

static int lut_index(struct switchtec_ntb *sndev, int mw_idx)
{
	return mw_idx - sndev->nr_direct_mw + sndev->nr_rsvd_luts;
}

static int peer_lut_index(struct switchtec_ntb *sndev, int mw_idx)
{
    ///dev_info(&sndev->stdev->dev, "%s(): mw_idx[%d] peer_nr_direct_mw[%d] "
    ///    "nr_rsvd_luts[%d] (Ln %d)\n",
    ///    __func__, mw_idx, sndev->peer_nr_direct_mw, sndev->nr_rsvd_luts,
    ///    __LINE__);  // KW_DB
        
	return mw_idx - sndev->peer_nr_direct_mw + sndev->nr_rsvd_luts;
}

static int switchtec_ntb_mw_get_align(struct ntb_dev *ntb, int pidx,
				      int widx, resource_size_t *addr_align,
				      resource_size_t *size_align,
				      resource_size_t *size_max)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	int lut;
	resource_size_t size;

    //dev_info(&sndev->stdev->dev, "%s(): Entered (Ln %d)\n",
    //////    __func__, __LINE__); // KW_DB
	if (pidx != NTB_DEF_PEER_IDX)
    {
        dev_err(&sndev->stdev->dev, "%s(): pidx[%d] NTB_DEF_PEER_IDX[%d] (Ln %d)\n",
            __func__, pidx, NTB_DEF_PEER_IDX, __LINE__);  // KW_DB        
		return -EINVAL;
    }

	lut = widx >= sndev->peer_nr_direct_mw;
	size = ioread64(&sndev->peer_shared->mw_sizes[widx]);

    ////dev_info(&sndev->stdev->dev, "%s(): lut[%d] size[%lld] (Ln %d)\n",
    /////    __func__, lut, size, __LINE__);  // KW_DB
	if (size == 0)
		return -EINVAL;

	if (addr_align)
		*addr_align = lut ? size : SZ_4K;

	if (size_align)
		*size_align = lut ? size : SZ_4K;

	if (size_max)
		*size_max = size;

	return 0;
}

static void switchtec_ntb_mw_clr_direct(struct switchtec_ntb *sndev, int idx)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	int bar = sndev->peer_direct_mw_to_bar[idx];
	u32 ctl_val;

	ctl_val = ioread32(&ctl->bar_entry[bar].ctl);
	ctl_val &= ~NTB_CTRL_BAR_DIR_WIN_EN;
	iowrite32(ctl_val, &ctl->bar_entry[bar].ctl);
	iowrite32(0, &ctl->bar_entry[bar].win_size);
	iowrite32(0, &ctl->bar_ext_entry[bar].win_size);
	iowrite64(sndev->self_partition, &ctl->bar_entry[bar].xlate_addr);
}

static void switchtec_ntb_mw_clr_lut(struct switchtec_ntb *sndev, int idx)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;

    printk("\n");  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): ctl(%p) is mmio_peer_ctrl.(Ln %d)\n", __func__, ctl, __LINE__);  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): idx[%d] value[0] (Ln %d)\n",
        __func__, idx, __LINE__);  // KW_DB
	iowrite64(0, &ctl->lut_entry[peer_lut_index(sndev, idx)]);
}

static void switchtec_ntb_mw_set_direct(struct switchtec_ntb   *sndev, 
                                        int                     idx,
					                    dma_addr_t              addr,
                                        resource_size_t         size)
{
	int xlate_pos = ilog2(size);
	int bar = sndev->peer_direct_mw_to_bar[idx];
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	u32 ctl_val;

	ctl_val = ioread32(&ctl->bar_entry[bar].ctl);
	ctl_val |= NTB_CTRL_BAR_DIR_WIN_EN;

	iowrite32(ctl_val, &ctl->bar_entry[bar].ctl);
	iowrite32(xlate_pos | (lower_32_bits(size) & 0xFFFFF000),
		  &ctl->bar_entry[bar].win_size);
	iowrite32(upper_32_bits(size), &ctl->bar_ext_entry[bar].win_size);
	iowrite64(sndev->self_partition | addr,
		  &ctl->bar_entry[bar].xlate_addr);
          
    dev_info(&sndev->stdev->dev, "%s(): BAR[%d] win_size[%lld].(Ln %d)\n",
        __func__, bar, size, __LINE__);  // KW_DB
}

static void switchtec_ntb_mw_set_lut(struct switchtec_ntb  *sndev,
                                     int                    idx,
				                     dma_addr_t             addr, 
                                     resource_size_t        size)
{
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;

    ///printk("\n");  // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): ctl(%p) is mmio_peer_ctrl.(Ln %d)\n",
        ///__func__, ctl, __LINE__);  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): idx[%d] value[x%llx] (Ln %d)\n",
        __func__, idx, (NTB_CTRL_LUT_EN | (sndev->self_partition << 1) | addr), __LINE__);  // KW_DB
        
	iowrite64((NTB_CTRL_LUT_EN | (sndev->self_partition << 1) | addr),
		  &ctl->lut_entry[peer_lut_index(sndev, idx)]);
}

static int switchtec_ntb_mw_set_trans(struct ntb_dev *ntb, int pidx, int widx,
				      dma_addr_t addr, resource_size_t size)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_peer_ctrl;
	int xlate_pos = ilog2(size);
	int nr_direct_mw = sndev->peer_nr_direct_mw;
	int rc;

    dev_info(&sndev->stdev->dev, "%s(): nr_direct_mw[%d] "
        "pidx[%d] widx[%d] (Ln %d)\n",
        __func__, sndev->nr_direct_mw, pidx, widx, __LINE__); // KW_DB
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	dev_dbg(&sndev->stdev->dev, "MW %d: part %d addr %pad size %pap\n",
		widx, pidx, &addr, &size);

	if (widx >= switchtec_ntb_mw_count(ntb, pidx))
    {
        dev_err(&sndev->stdev->dev, "%s(): Here (Ln %d)\n",
        __func__, __LINE__); // KW_DB
		return -EINVAL;
    }

	if (size != 0 && xlate_pos < 12)
    {
        dev_err(&sndev->stdev->dev, "%s(): size[0x%llX] xlate_pos[%d] (Ln %d)\n",
        __func__, size, xlate_pos, __LINE__); // KW_DB
		return -EINVAL;
    }

    // KW_DB->
    dev_info(&sndev->stdev->dev, "%s():  pidx[%d] widx[%d] addr[0x%llX] xlate_pos[%d] size[0x%llX]\n",
		__func__, pidx, widx, addr, xlate_pos, size);
        
    // if (!addr || xlate_pos < 0)  // KW_DB_20241211
    if (xlate_pos < 0)              // KW_DB_20241211   - Allow mapping to address 0x00000000.
    {
        dev_err(&sndev->stdev->dev, "pidx[%d] widx[%d] addr[0x%llX] xlate_pos[%d] size[%lld]\n",
            pidx, widx, addr, xlate_pos, size);
        return -EINVAL;
    }
    // KW_DB<-
    
	if (!IS_ALIGNED(addr, BIT_ULL(xlate_pos))) {
		/*
		 * In certain circumstances we can get a buffer that is
		 * not aligned to its size. (Most of the time
		 * dma_alloc_coherent ensures this). This can happen when
		 * using large buffers allocated by the CMA
		 * (see CMA_CONFIG_ALIGNMENT)
		 */
		dev_err(&sndev->stdev->dev,
			"ERROR: Memory window address is not aligned to it's size!\n");
		return -EINVAL;
	}

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	if (size == 0) {
		if (widx < nr_direct_mw)
			switchtec_ntb_mw_clr_direct(sndev, widx);
		else
			switchtec_ntb_mw_clr_lut(sndev, widx);
	} else {
		if (widx < nr_direct_mw)
			switchtec_ntb_mw_set_direct(sndev, widx, addr, size);
		else
			switchtec_ntb_mw_set_lut(sndev, widx, addr, size);
	}

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);

	if (rc == -EIO) {
		dev_err(&sndev->stdev->dev,
			"Hardware reported an error configuring mw %d: %08x\n",
			widx, ioread32(&ctl->bar_error));

		if (widx < nr_direct_mw)
			switchtec_ntb_mw_clr_direct(sndev, widx);
		else
			switchtec_ntb_mw_clr_lut(sndev, widx);

		switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				      NTB_CTRL_PART_STATUS_NORMAL);
	}

	return rc;
}

static int switchtec_ntb_peer_mw_count(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	int nr_lut_mw = sndev->nr_lut_mw - sndev->nr_rsvd_luts;

    dev_info(&sndev->stdev->dev, "%s(): nr_lut_mw[%d] (Ln %d)\n",
        __func__, nr_lut_mw, __LINE__); // KW_DB
        
	return sndev->nr_direct_mw + (use_lut_mws ? nr_lut_mw : 0);
}

static int switchtec_ntb_direct_get_addr(struct switchtec_ntb *sndev,
					 int idx, phys_addr_t *base,
					 resource_size_t *size)
{
	int bar = sndev->direct_mw_to_bar[idx];
	size_t offset = 0;

	if (bar < 0)
		return -EINVAL;

    ///dev_info(&sndev->stdev->dev, "%s(): size[%lld] (Ln %d)\n",
    ///    __func__, *size, __LINE__); // KW_DB
        
	if (idx == 0) {
		/*
		 * This is the direct BAR shared with the LUTs
		 * which means the actual window will be offset
		 * by the size of all the LUT entries.
		 */

		offset = LUT_SIZE * sndev->nr_lut_mw;
	}

	if (base)
		*base = pci_resource_start(sndev->ntb.pdev, bar) + offset;


        
	if (size) {
		*size = pci_resource_len(sndev->ntb.pdev, bar) - offset;
		if (offset && *size > offset)
			*size = offset;

		if (*size > max_mw_size)
			*size = max_mw_size;
	}

    dev_info(&sndev->stdev->dev, "%s(): base [x%llX] offset[x%lX] size[%lld] (Ln %d)\n",
        __func__, *base, offset, *size, __LINE__); // KW_DB
        
	return 0;
}

static int switchtec_ntb_lut_get_addr(struct switchtec_ntb *sndev,
				      int idx, phys_addr_t *base,
				      resource_size_t *size)
{
	int bar = sndev->direct_mw_to_bar[0];
	int offset;

	offset = LUT_SIZE * lut_index(sndev, idx);

	if (base)
		*base = pci_resource_start(sndev->ntb.pdev, bar) + offset;

	if (size)
		*size = LUT_SIZE;

	return 0;
}

static int switchtec_ntb_peer_mw_get_addr(struct ntb_dev *ntb, int idx,
					  phys_addr_t *base,
					  resource_size_t *size)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

    dev_info(&sndev->stdev->dev, "%s(): idx[%d].(Ln %d)\n",
        __func__, idx, __LINE__);  // KW_DB
	if (idx < sndev->nr_direct_mw)
    {
        dev_info(&sndev->stdev->dev, "%s(): Calling switchtec_ntb_direct_get_addr() idx[%d].(Ln %d)\n",
            __func__, idx, __LINE__);  // KW_DB
		return switchtec_ntb_direct_get_addr(sndev, idx, base, size);
    }
	else if (idx < switchtec_ntb_peer_mw_count(ntb))
		return switchtec_ntb_lut_get_addr(sndev, idx, base, size);
	else
		return -EINVAL;
}

static void switchtec_ntb_part_link_speed(struct switchtec_ntb *sndev,
					  int partition,
					  enum ntb_speed *speed,
					  enum ntb_width *width)
{
	struct switchtec_dev *stdev = sndev->stdev;
	u32 pff;
	u32 linksta;

	pff = ioread32(&stdev->mmio_part_cfg_all[partition].vep_pff_inst_id);
	pff &= 0xFF;
	if (pff == 0xFF) {
		dev_warn(&sndev->stdev->dev,
			 "Invalid pff, setting speed/width to 0");
		*speed = 0;
		*width = 0;
		return;
	}

	linksta = ioread32(&stdev->mmio_pff_csr[pff].pci_cap_region[13]);

	if (speed)
		*speed = (linksta >> 16) & 0xF;

	if (width)
		*width = (linksta >> 20) & 0x3F;
}

static void switchtec_ntb_set_link_speed(struct switchtec_ntb *sndev)
{
	enum ntb_speed self_speed, peer_speed;
	enum ntb_width self_width, peer_width;

	if (!sndev->link_is_up) {
		sndev->link_speed = NTB_SPEED_NONE;
		sndev->link_width = NTB_WIDTH_NONE;
		return;
	}

	switchtec_ntb_part_link_speed(sndev, sndev->self_partition,
				      &self_speed, &self_width);
	switchtec_ntb_part_link_speed(sndev, sndev->peer_partition,
				      &peer_speed, &peer_width);

	sndev->link_speed = min(self_speed, peer_speed);
	sndev->link_width = min(self_width, peer_width);
}

static int crosslink_is_enabled(struct switchtec_ntb *sndev)
{
	struct ntb_info_regs __iomem *inf = sndev->mmio_ntb;

	return ioread8(&inf->ntp_info[sndev->peer_partition].xlink_enabled);
}

static void crosslink_init_dbmsgs(struct switchtec_ntb *sndev)
{
	int i;
	u32 msg_map = 0;

	if (!crosslink_is_enabled(sndev))
		return;

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_peer_dbmsg->imsg); i++) {
		int m = i | sndev->self_partition << 2;

		msg_map |= m << i * 8;
	}

	iowrite32(msg_map, &sndev->mmio_peer_dbmsg->msg_map);
	iowrite64(sndev->db_valid_mask << sndev->db_peer_shift,
		  &sndev->mmio_peer_dbmsg->odb_mask);
}

enum switchtec_msg {
	LINK_MESSAGE = 0,
	MSG_LINK_UP = 1,
	MSG_LINK_DOWN = 2,
	MSG_CHECK_LINK = 3,
	MSG_LINK_FORCE_DOWN = 4,
};

static int switchtec_ntb_reinit_peer(struct switchtec_ntb *sndev);

static int crosslink_setup_req_ids(struct switchtec_ntb *sndev,
		struct ntb_ctrl_regs __iomem *mmio_ctrl);

static void switchtec_ntb_link_status_update(struct switchtec_ntb *sndev)
{
	int link_sta;
	int old = sndev->link_is_up;
	u64 peer;

	link_sta = sndev->self_shared->link_sta;
    dev_info(&sndev->stdev->dev, "%s(): link_sta[x%X] link was up[%d] crosslink_is_enabled[%d] (Ln %d)\n",
        __func__, link_sta, old, crosslink_is_enabled(sndev), __LINE__);  // KW_DB
    
	if (link_sta) {
		if (!sndev->link_is_up && crosslink_is_enabled(sndev))
        {   // KW_DB
            dev_info(&sndev->stdev->dev, "%s(): Calling crosslink_setup_req_ids() (Ln %d)\n",
                __func__, __LINE__);  // KW_DB
			crosslink_setup_req_ids(sndev, sndev->mmio_xlink_peer_ctrl);
        }   // KW_DB

		peer = ioread64(&sndev->peer_shared->magic);

		if ((peer & 0xFFFFFFFF) == SWITCHTEC_NTB_MAGIC)
			link_sta = peer >> 32;
		else
			link_sta = 0;
	}

	sndev->link_is_up = link_sta;
    dev_info(&sndev->stdev->dev, "%s(): link_sta[x%X] link_is_up[%d] crosslink_is_enabled[%d] (Ln %d)\n",
        __func__, link_sta, sndev->link_is_up, crosslink_is_enabled(sndev), __LINE__);  // KW_DB
	switchtec_ntb_set_link_speed(sndev);

    dev_info(&sndev->stdev->dev, "%s(): link_sta[%d] old[%d] (Ln %d)\n",
        __func__, link_sta, old, __LINE__);  // KW_DB
                
	if (link_sta != old) {
		switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_CHECK_LINK);
		ntb_link_event(&sndev->ntb);
		dev_info(&sndev->stdev->dev, "ntb link %s\n",
			 link_sta ? "up" : "down");

		if (link_sta)
			crosslink_init_dbmsgs(sndev);
	}
}

static void check_link_status_work(struct work_struct *work)
{
	struct switchtec_ntb *sndev;

	sndev = container_of(work, struct switchtec_ntb,
			     check_link_status_work);

	if (sndev->link_force_down) {
		sndev->link_force_down = false;
		switchtec_ntb_reinit_peer(sndev);

		if (sndev->link_is_up) {
			sndev->link_is_up = 0;
			ntb_link_event(&sndev->ntb);
			dev_info(&sndev->stdev->dev, "ntb link forced down\n");
		}

		return;
	}

	switchtec_ntb_link_status_update(sndev);
}

static void switchtec_ntb_check_link(struct switchtec_ntb *sndev,
				     enum switchtec_msg msg)
{
	if (msg == MSG_LINK_FORCE_DOWN)
		sndev->link_force_down = true;

    dev_info(&sndev->stdev->dev, "%s(): Calling schedule_work() (Ln %d)\n", __func__, __LINE__);  // KW_DB
	schedule_work(&sndev->check_link_status_work);
}

static void switchtec_ntb_link_notification(struct switchtec_dev *stdev)
{
	struct switchtec_ntb *sndev = stdev->sndev;

	switchtec_ntb_check_link(sndev, MSG_CHECK_LINK);
}

static u64 switchtec_ntb_link_is_up(struct ntb_dev *ntb,
				    enum ntb_speed *speed,
				    enum ntb_width *width)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (speed)
		*speed = sndev->link_speed;
	if (width)
		*width = sndev->link_width;

	return sndev->link_is_up;
}

static int switchtec_ntb_link_enable(struct ntb_dev *ntb,
				     enum ntb_speed max_speed,
				     enum ntb_width max_width)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	dev_dbg(&sndev->stdev->dev, "enabling link\n");

	sndev->self_shared->link_sta = 1;
    ///switchtec_ntb_link_status_update(sndev);    // KW_DB
	switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_LINK_UP);
    switchtec_ntb_link_status_update(sndev);    // KW_DB

	return 0;
}

static int switchtec_ntb_link_disable(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	dev_dbg(&sndev->stdev->dev, "disabling link\n");

	sndev->self_shared->link_sta = 0;
	switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_LINK_DOWN);

	switchtec_ntb_link_status_update(sndev);

	return 0;
}

static u64 switchtec_ntb_db_valid_mask(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	return sndev->db_valid_mask;
}

static int switchtec_ntb_db_vector_count(struct ntb_dev *ntb)
{
	return 1;
}

static u64 switchtec_ntb_db_vector_mask(struct ntb_dev *ntb, int db_vector)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (db_vector < 0 || db_vector > 1)
		return 0;

	return sndev->db_valid_mask;
}

static u64 switchtec_ntb_db_read(struct ntb_dev *ntb)
{
	u64 ret;
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	ret = ioread64(&sndev->mmio_self_dbmsg->idb) >> sndev->db_shift;

	return ret & sndev->db_valid_mask;
}

static int switchtec_ntb_db_clear(struct ntb_dev *ntb, u64 db_bits)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	iowrite64(db_bits << sndev->db_shift, &sndev->mmio_self_dbmsg->idb);

	return 0;
}

static int switchtec_ntb_db_set_mask(struct ntb_dev *ntb, u64 db_bits)
{
	unsigned long irqflags;
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (db_bits & ~sndev->db_valid_mask)
		return -EINVAL;

	spin_lock_irqsave(&sndev->db_mask_lock, irqflags);

	sndev->db_mask |= db_bits << sndev->db_shift;
	iowrite64(~sndev->db_mask, &sndev->mmio_self_dbmsg->idb_mask);

	spin_unlock_irqrestore(&sndev->db_mask_lock, irqflags);

	return 0;
}

static int switchtec_ntb_db_clear_mask(struct ntb_dev *ntb, u64 db_bits)
{
	unsigned long irqflags;
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	if (db_bits & ~sndev->db_valid_mask)
		return -EINVAL;

	spin_lock_irqsave(&sndev->db_mask_lock, irqflags);

	sndev->db_mask &= ~(db_bits << sndev->db_shift);
	iowrite64(~sndev->db_mask, &sndev->mmio_self_dbmsg->idb_mask);

	spin_unlock_irqrestore(&sndev->db_mask_lock, irqflags);

	return 0;
}

static u64 switchtec_ntb_db_read_mask(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	return (sndev->db_mask >> sndev->db_shift) & sndev->db_valid_mask;
}

static int switchtec_ntb_peer_db_addr(struct ntb_dev *ntb,
				      phys_addr_t *db_addr,
				      resource_size_t *db_size,
				      u64 *db_data,
				      int db_bit)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	unsigned long offset;

	if (unlikely(db_bit >= BITS_PER_LONG_LONG))
		return -EINVAL;

	offset = (unsigned long)sndev->mmio_peer_dbmsg->odb -
		(unsigned long)sndev->stdev->mmio;

	offset += sndev->db_shift / 8;

	if (db_addr)
		*db_addr = pci_resource_start(ntb->pdev, 0) + offset;
	if (db_size)
		*db_size = sizeof(u32);
	if (db_data)
		*db_data = BIT_ULL(db_bit) << sndev->db_peer_shift;

	return 0;
}

static int switchtec_ntb_peer_db_set(struct ntb_dev *ntb, u64 db_bits)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	iowrite64(db_bits << sndev->db_peer_shift,
		  &sndev->mmio_peer_dbmsg->odb);

	return 0;
}

static int switchtec_ntb_spad_count(struct ntb_dev *ntb)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

	return ARRAY_SIZE(sndev->self_shared->spad);
}

static u32 switchtec_ntb_spad_read(struct ntb_dev *ntb, int idx)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
void *ptr; // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): idx[%d].(Ln %d)\n", __func__, idx, __LINE__);  // KW_DB
    
	if (idx < 0 || idx >= ARRAY_SIZE(sndev->self_shared->spad))
		return 0;

	if (!sndev->self_shared)
		return 0;

    ///ptr = sndev->self_shared; // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): self_shared(%p).(Ln %d)\n", __func__, ptr, __LINE__);  // KW_DB
    ptr = sndev->self_shared->spad; // KW_DB
    
    ///dev_info(&sndev->stdev->dev, "%s(): spad(%p) p(%p).(Ln %d)\n", __func__, &(sndev->self_shared->spad[idx]), sndev->self_shared->spad, __LINE__);  // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): val1[x%X] Val2[x%X] (Ln %d)\n", __func__, *(u32*)(ptr+4), *(u32*)(ptr+8), __LINE__);  // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): idx[%d] val[x%X].(Ln %d)\n", __func__, idx, sndev->self_shared->spad[idx], __LINE__);  // KW_DB
	return sndev->self_shared->spad[idx];
}

static int switchtec_ntb_spad_write(struct ntb_dev *ntb, int idx, u32 val)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

    dev_info(&sndev->stdev->dev, "%s(): idx[%d] val[x%X].(Ln %d)\n",
        __func__, idx, val, __LINE__);  // KW_DB
    
	if (idx < 0 || idx >= ARRAY_SIZE(sndev->self_shared->spad))
		return -EINVAL;

	if (!sndev->self_shared)
		return -EIO;

	sndev->self_shared->spad[idx] = val;

	return 0;
}

static u32 switchtec_ntb_peer_spad_read(struct ntb_dev *ntb, int pidx,
					int sidx)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
    u32 val;    // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): pidx[%d].(Ln %d)\n", __func__, pidx, __LINE__);  // KW_DB
    
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	if (sidx < 0 || sidx >= ARRAY_SIZE(sndev->peer_shared->spad))
		return 0;

	if (!sndev->peer_shared)
		return 0;

    val = ioread32(&sndev->peer_shared->spad[sidx]); // KW_DB
	//return ioread32(&sndev->peer_shared->spad[sidx]); // KW_DB
    ///dev_info(&sndev->stdev->dev, "%s(): pidx[%d] val[x%X].(Ln %d)\n", __func__, pidx, val, __LINE__);  // KW_DB
    return val; // KW_DB
}

static int switchtec_ntb_peer_spad_write(struct ntb_dev *ntb, int pidx,
					 int sidx, u32 val)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);

    ///dev_info(&sndev->stdev->dev, "%s(): pidx[%d] val[x%X].(Ln %d)\n", __func__, pidx, val, __LINE__);  // KW_DB
    
	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	if (sidx < 0 || sidx >= ARRAY_SIZE(sndev->peer_shared->spad))
		return -EINVAL;

	if (!sndev->peer_shared)
		return -EIO;

	iowrite32(val, &sndev->peer_shared->spad[sidx]);

	return 0;
}

static int switchtec_ntb_peer_spad_addr(struct ntb_dev *ntb, int pidx,
					int sidx, phys_addr_t *spad_addr)
{
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	unsigned long offset;

	if (pidx != NTB_DEF_PEER_IDX)
		return -EINVAL;

	offset = (unsigned long)&sndev->peer_shared->spad[sidx] -
		(unsigned long)sndev->stdev->mmio;

	if (spad_addr)
		*spad_addr = pci_resource_start(ntb->pdev, 0) + offset;

	return 0;
}

static const struct ntb_dev_ops switchtec_ntb_ops = {
	.mw_count		= switchtec_ntb_mw_count,
	.mw_get_align		= switchtec_ntb_mw_get_align,
	.mw_set_trans		= switchtec_ntb_mw_set_trans,
	.peer_mw_count		= switchtec_ntb_peer_mw_count,
	.peer_mw_get_addr	= switchtec_ntb_peer_mw_get_addr,
	.link_is_up		= switchtec_ntb_link_is_up,
	.link_enable		= switchtec_ntb_link_enable,
	.link_disable		= switchtec_ntb_link_disable,
	.db_valid_mask		= switchtec_ntb_db_valid_mask,
	.db_vector_count	= switchtec_ntb_db_vector_count,
	.db_vector_mask		= switchtec_ntb_db_vector_mask,
	.db_read		= switchtec_ntb_db_read,
	.db_clear		= switchtec_ntb_db_clear,
	.db_set_mask		= switchtec_ntb_db_set_mask,
	.db_clear_mask		= switchtec_ntb_db_clear_mask,
	.db_read_mask		= switchtec_ntb_db_read_mask,
	.peer_db_addr		= switchtec_ntb_peer_db_addr,
	.peer_db_set		= switchtec_ntb_peer_db_set,
	.spad_count		= switchtec_ntb_spad_count,
	.spad_read		= switchtec_ntb_spad_read,
	.spad_write		= switchtec_ntb_spad_write,
	.peer_spad_read		= switchtec_ntb_peer_spad_read,
	.peer_spad_write	= switchtec_ntb_peer_spad_write,
	.peer_spad_addr		= switchtec_ntb_peer_spad_addr,
};

static int switchtec_ntb_init_sndev(struct switchtec_ntb *sndev)
{
	u64 tpart_vec;
	int self;
	u64 part_map;

	sndev->ntb.pdev = sndev->stdev->pdev;
	sndev->ntb.topo = NTB_TOPO_SWITCH;
	sndev->ntb.ops = &switchtec_ntb_ops;

    dev_info(&sndev->stdev->dev, "%s(): INIT_WORK(check_link_status_work) (Ln %d)\n", __func__, __LINE__);  // KW_DB
	INIT_WORK(&sndev->check_link_status_work, check_link_status_work);
	sndev->link_force_down = false;

	sndev->self_partition = sndev->stdev->partition;
    dev_info(&sndev->stdev->dev, "%s(): self_partition=%d (Ln %d)\n", __func__, sndev->self_partition, __LINE__);  // KW_DB

	sndev->mmio_ntb = sndev->stdev->mmio_ntb;

	self = sndev->self_partition;
	tpart_vec = ioread32(&sndev->mmio_ntb->ntp_info[self].target_part_high);
	tpart_vec <<= 32;
	tpart_vec |= ioread32(&sndev->mmio_ntb->ntp_info[self].target_part_low);
    dev_info(&sndev->stdev->dev, "%s(): tpart_vec=0x%llX (Ln %d)\n", __func__, tpart_vec, __LINE__);  // KW_DB
	part_map = ioread32(&sndev->mmio_ntb->ep_map_high);
    dev_info(&sndev->stdev->dev, "%s(): ep_map_high=%d (Ln %d)\n", __func__, ioread32(&sndev->mmio_ntb->ep_map_high), __LINE__);  // KW_DB
	part_map <<= 32;
	part_map |= ioread32(&sndev->mmio_ntb->ep_map_low);
    dev_info(&sndev->stdev->dev, "%s(): ep_map_low=%d (Ln %d)\n", __func__, ioread32(&sndev->mmio_ntb->ep_map_low), __LINE__);  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): part_map=0x%llX (Ln %d)\n", __func__, part_map, __LINE__);  // KW_DB
    
	tpart_vec &= part_map;
    dev_info(&sndev->stdev->dev, "%s(): tpart_vec=0x%llX (Ln %d)\n", __func__, tpart_vec, __LINE__);  // KW_DB
    
	part_map &= ~(1 << sndev->self_partition);
    dev_info(&sndev->stdev->dev, "%s(): Final part_map=0x%llX (Ln %d)\n", __func__, part_map, __LINE__);  // KW_DB
    
    /*
     * __ffs64() is 0-based // KW_DB
     * fls64() is 1-based
     */

	if (!tpart_vec) {
		if (sndev->stdev->partition_count != 2) {
			dev_err(&sndev->stdev->dev,
				"ntb target partition not defined\n");
			return -ENODEV;
		}

		if (!part_map) {
			dev_err(&sndev->stdev->dev,
				"peer partition is not NT partition\n");
			return -ENODEV;
		}

		sndev->peer_partition = __ffs64(part_map);
        dev_info(&sndev->stdev->dev, "%s(): peer_partition=%d (Ln %d)\n", __func__, sndev->peer_partition, __LINE__);  // KW_DB
	} else {
        dev_info(&sndev->stdev->dev, "%s(): __ffs64(tpart_vec) = %ld (Ln %d)\n", __func__, __ffs64(tpart_vec) , __LINE__);  // KW_DB
        dev_info(&sndev->stdev->dev, "%s():   fls64(tpart_vec) = %d (Ln %d)\n", __func__, fls64(tpart_vec) , __LINE__);  // KW_DB
        #if 0   // KW_DB_20250630
		if (__ffs64(tpart_vec) != (fls64(tpart_vec) - 1)) {
			dev_err(&sndev->stdev->dev,
				"ntb driver only supports 1 pair of 1-1 ntb mapping\n");
			return -ENODEV;
		}
        #endif  // KW_DB_20250630
		sndev->peer_partition = __ffs64(tpart_vec);
        dev_info(&sndev->stdev->dev, "%s(): peer_partition=%d (Ln %d)\n", __func__, sndev->peer_partition, __LINE__);  // KW_DB
        // KW_DB_20250710->
        //sndev->peer_partition = 2;
        //dev_info(&sndev->stdev->dev, "%s(): Force peer_partition=%d (Ln %d)\n", __func__, sndev->peer_partition, __LINE__);  // KW_DB
        // KW_DB_20250710<-
		if (!(part_map & (1ULL << sndev->peer_partition))) {
			dev_err(&sndev->stdev->dev,
				"ntb target partition is not NT partition\n");
			return -ENODEV;
		}
	}

	dev_dbg(&sndev->stdev->dev, "Partition ID %d of %d\n",
		sndev->self_partition, sndev->stdev->partition_count);
	
    // KW_DB->
    dev_dbg(&sndev->stdev->dev, "Self Partition ID %d of %d\n",
		sndev->self_partition, sndev->stdev->partition_count);
    dev_dbg(&sndev->stdev->dev, "Peer Partition ID %d of %d\n",
		sndev->peer_partition, sndev->stdev->partition_count);        
    // KW_DB<-
        
	sndev->mmio_ctrl = (void * __iomem)sndev->mmio_ntb +
		SWITCHTEC_NTB_REG_CTRL_OFFSET;
	sndev->mmio_dbmsg = (void * __iomem)sndev->mmio_ntb +
		SWITCHTEC_NTB_REG_DBMSG_OFFSET;

	sndev->mmio_self_ctrl = &sndev->mmio_ctrl[sndev->self_partition];
	sndev->mmio_peer_ctrl = &sndev->mmio_ctrl[sndev->peer_partition];
	sndev->mmio_self_dbmsg = &sndev->mmio_dbmsg[sndev->self_partition];
	sndev->mmio_peer_dbmsg = sndev->mmio_self_dbmsg;

	return 0;
}

static int config_rsvd_lut_win(struct switchtec_ntb *sndev,
			                   struct ntb_ctrl_regs __iomem *ctl,
			                   int lut_idx, int partition, u64 addr)
{
	int peer_bar = sndev->peer_direct_mw_to_bar[0];
	u32 ctl_val;
	int rc;
    
    // KW_DB->
    printk("\n"); 
    dev_info(&sndev->stdev->dev, "%s(): ctl(%p).(Ln %d)\n", __func__, ctl, __LINE__);
    dev_info(&sndev->stdev->dev, "%s(): lut_idx[%d] partition[%d] addr[x%llx] " \
        "peer_bar[0x%x]  (Ln %d)\n",
        __func__, lut_idx, partition, addr, peer_bar, __LINE__);
    // KW_DB<-
	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_LOCK,
				               NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	ctl_val = ioread32(&ctl->bar_entry[peer_bar].ctl);
	ctl_val &= 0xFF;
	ctl_val |= NTB_CTRL_BAR_LUT_WIN_EN;
	ctl_val |= ilog2(LUT_SIZE) << 8;
	ctl_val |= (sndev->nr_lut_mw - 1) << 14;
    dev_info(&sndev->stdev->dev, "%s(): ctl_val[x%X] value[x%llx] p(Ln %d)\n",
        __func__, ctl_val, (NTB_CTRL_LUT_EN | (partition << 1) | addr), __LINE__);  // KW_DB

	iowrite32(ctl_val, &ctl->bar_entry[peer_bar].ctl);

	iowrite64((NTB_CTRL_LUT_EN | (partition << 1) | addr),
		  &ctl->lut_entry[lut_idx]);

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				               NTB_CTRL_PART_STATUS_NORMAL);
	if (rc) {
		u32 bar_error, lut_error;

		bar_error = ioread32(&ctl->bar_error);
		lut_error = ioread32(&ctl->lut_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up reserved lut window: %08x / %08x\n",
			bar_error, lut_error);
		return rc;
	}

	return 0;
}

static int add_req_id(struct switchtec_ntb *sndev,
		      struct ntb_ctrl_regs __iomem *mmio_ctrl, int req_id)
{
	int i, rc = 0;
	int slot = -1;
	u32 error;
	int table_size;
	u32 proxy_id = 0;
	bool added = true;

	table_size = ioread16(&mmio_ctrl->req_id_table_size);

	rc = switchtec_ntb_part_op(sndev, mmio_ctrl,
				   NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
    {
        dev_info(&sndev->stdev->dev, "%s(): rc[%d] (Ln %d)\n", __func__, rc, __LINE__);  // KW_DB
		return rc;
    }

	for (i = 0; i < table_size; i++) {
		proxy_id = ioread32(&mmio_ctrl->req_id_table[i]);

		if (!(proxy_id & NTB_CTRL_REQ_ID_EN) && slot == -1)
			slot = i;

		if (proxy_id & NTB_CTRL_REQ_ID_EN &&
		    proxy_id >> 16 == req_id) {
            dev_info(&sndev->stdev->dev, "%s(): %d: proxy_id[%d] req_id[%d] (Ln %d)\n",
                __func__, i, proxy_id, req_id, __LINE__);  // KW_DB
			goto unlock_exit;
		}
	}

	if (slot == -1) {
		dev_err(&sndev->stdev->dev,
			"Not enough requester IDs available.\n");
		added = false;
	} else {
		iowrite32(req_id << 16 | NTB_CTRL_REQ_ID_EN,
			  &mmio_ctrl->req_id_table[slot]);

		proxy_id = ioread32(&mmio_ctrl->req_id_table[slot]);
		dev_dbg(&sndev->stdev->dev,
			"Requester ID %02X:%02X.%X -> BB:%02X.%X\n",
			req_id >> 8, (req_id >> 3) & 0x1F,
			req_id & 0x7, (proxy_id >> 4) & 0x1F,
			(proxy_id >> 1) & 0x7);
		/// dev_info(&sndev->stdev->dev,
			/// "Requester ID %02X:%02X.%X -> BB:%02X.%X\n",
			/// req_id >> 8, (req_id >> 3) & 0x1F,
			/// req_id & 0x7, (proxy_id >> 4) & 0x1F,
			/// (proxy_id >> 1) & 0x7); // KW_DB
		added = true;
	}

unlock_exit:
	rc = switchtec_ntb_part_op(sndev, mmio_ctrl,
				   NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);

	if (rc == -EIO) {
		error = ioread32(&mmio_ctrl->req_id_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up the requester ID table: %08x\n",
			error);
	}

	if (!added)
    {
        dev_info(&sndev->stdev->dev, "%s(): -EFAULT (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -EFAULT;
    }

	return 0;
}

static int del_req_id(struct switchtec_ntb *sndev,
		      struct ntb_ctrl_regs __iomem *mmio_ctrl, int req_id)
{
	int i, rc = 0;
	u32 error;
	int table_size;
	u32 rid;
	bool deleted = true;

	table_size = ioread16(&mmio_ctrl->req_id_table_size);

	rc = switchtec_ntb_part_op(sndev, mmio_ctrl,
				   NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	for (i = 0; i < table_size; i++) {
		rid = ioread32(&mmio_ctrl->req_id_table[i]);

		if (!(rid & NTB_CTRL_REQ_ID_EN))
			continue;

		rid >>= 16;
		if (rid == req_id) {
			iowrite32(0, &mmio_ctrl->req_id_table[i]);
			break;
		}
	}

	if (i == table_size) {
		dev_err(&sndev->stdev->dev,
			"Requester ID %02X:%02X.%X not in the table.\n",
			PCI_BUS_NUM(req_id), PCI_SLOT(req_id),
			PCI_FUNC(req_id));
		deleted = false;
	}

	rc = switchtec_ntb_part_op(sndev, mmio_ctrl,
				   NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);

	if (rc == -EIO) {
		error = ioread32(&mmio_ctrl->req_id_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up the requester ID table: %08x\n",
			error);
	}

	if (!deleted)
		return -ENXIO;

	return 0;
}

static int clr_req_ids(struct switchtec_ntb *sndev,
		       struct ntb_ctrl_regs __iomem *mmio_ctrl)
{
	int i, rc = 0;
	u32 error;
	int table_size;

	table_size = ioread16(&mmio_ctrl->req_id_table_size);

	rc = switchtec_ntb_part_op(sndev, mmio_ctrl,
				   NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	for (i = 0; i < table_size; i++)
		iowrite32(0, &mmio_ctrl->req_id_table[i]);

	rc = switchtec_ntb_part_op(sndev, mmio_ctrl,
				   NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);

	if (rc == -EIO) {
		error = ioread32(&mmio_ctrl->req_id_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up the requester ID table: %08x\n",
			error);
	}

	return 0;
}

static int crosslink_setup_mws(struct switchtec_ntb *sndev,
			       int ntb_dbmsg_lut_idx,
			       int ntb_req_id_lut_idx,
			       u64 *mw_addrs, int mw_count)
{
	int rc, i;
	struct ntb_ctrl_regs __iomem *ctl = sndev->mmio_self_ctrl;
	u64 addr;
	size_t size, offset;
	int bar;
	int xlate_pos;
	u32 ctl_val;

    printk("\n");  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): ctl(%p) is mmio_self_ctrl.(Ln %d)\n", __func__, ctl, __LINE__);  // KW_DB
    dev_info(&sndev->stdev->dev, "%s(): ntb_dbmsg_lut_idx[%d] ntb_req_id_lut_idx[%d]\n\t  \
                                  mw_addrs[%p] mw_count[%d]  (Ln %d)\n",
        __func__, ntb_dbmsg_lut_idx, ntb_req_id_lut_idx, mw_addrs, mw_count, __LINE__);  // KW_DB

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_LOCK,
				   NTB_CTRL_PART_STATUS_LOCKED);
	if (rc)
		return rc;

	for (i = 0; i < sndev->nr_lut_mw; i++) {
		if (i == ntb_dbmsg_lut_idx || i == ntb_req_id_lut_idx)
			continue;

		addr = mw_addrs[0] + LUT_SIZE * i;

        ///dev_info(&sndev->stdev->dev, "%s(): i[%d] value[x%llx] (Ln %d)\n",
        ///    __func__, i, (NTB_CTRL_LUT_EN | (sndev->peer_partition << 1) | addr), __LINE__);  // KW_DB
		iowrite64((NTB_CTRL_LUT_EN | (sndev->peer_partition << 1) |
			   addr),
			  &ctl->lut_entry[i]);
	}

	sndev->nr_direct_mw = min_t(int, sndev->nr_direct_mw, mw_count);
    dev_info(&sndev->stdev->dev, "%s(): nr_direct_mw[%d] (Ln %d)\n",
        __func__, sndev->nr_direct_mw, __LINE__); // KW_DB

	for (i = 0; i < sndev->nr_direct_mw; i++) {
		bar = sndev->direct_mw_to_bar[i];
		offset = (i == 0) ? LUT_SIZE * sndev->nr_lut_mw : 0;
		addr = mw_addrs[i] + offset;
		size = pci_resource_len(sndev->ntb.pdev, bar) - offset;
		xlate_pos = ilog2(size);

		if (offset && size > offset)
			size = offset;

		ctl_val = ioread32(&ctl->bar_entry[bar].ctl);
		ctl_val |= NTB_CTRL_BAR_DIR_WIN_EN;

         dev_info(&sndev->stdev->dev, "%s(): i[%d] bar[%d] size[0x%lX] offset[0x%lX] (Ln %d)\n",
             __func__, i, bar, size, offset, __LINE__);  // KW_DB
         dev_info(&sndev->stdev->dev, "        addr[x%llx] xlate_pos[0x%X]\n",
             addr, xlate_pos);  // KW_DB
            
		iowrite32(ctl_val, &ctl->bar_entry[bar].ctl);
		iowrite32(xlate_pos | (lower_32_bits(size) & 0xFFFFF000),
			  &ctl->bar_entry[bar].win_size);
		iowrite32(upper_32_bits(size), &ctl->bar_ext_entry[bar].win_size);
		iowrite64(sndev->peer_partition | addr,
			  &ctl->bar_entry[bar].xlate_addr);
	}

	rc = switchtec_ntb_part_op(sndev, ctl, NTB_CTRL_PART_OP_CFG,
				   NTB_CTRL_PART_STATUS_NORMAL);
	if (rc) {
		u32 bar_error, lut_error;

		bar_error = ioread32(&ctl->bar_error);
		lut_error = ioread32(&ctl->lut_error);
		dev_err(&sndev->stdev->dev,
			"Error setting up cross link windows: %08x / %08x\n",
			bar_error, lut_error);
		return rc;
	}

	return 0;
}

static int crosslink_setup_req_ids(struct switchtec_ntb *sndev,
	struct ntb_ctrl_regs __iomem *mmio_ctrl)
{
	int i;
	u32 proxy_id;
	int table_size;
	int rc;

	table_size = ioread16(&mmio_ctrl->req_id_table_size);

	clr_req_ids(sndev, mmio_ctrl);

    dev_info(&sndev->stdev->dev, "%s(): Entered (Ln %d)\n", __func__, __LINE__);  // KW_DB

	for (i = 0; i < table_size; i++) {
		proxy_id = ioread32(&sndev->mmio_self_ctrl->req_id_table[i]);

		if (!(proxy_id & NTB_CTRL_REQ_ID_EN))
			continue;

		proxy_id = ((proxy_id >> 1) & 0xFF);
        dev_info(&sndev->stdev->dev, "%s(): Adding req_id[x%x] (Ln %d)\n", __func__, proxy_id, __LINE__);  // KW_DB
		rc = add_req_id(sndev, mmio_ctrl, proxy_id);
		if (rc)
			return rc;
	}

	return 0;
}

/*
 * In crosslink configuration there is a virtual partition in the
 * middle of the two switches. The BARs in this partition have to be
 * enumerated and assigned addresses.
 */
static int crosslink_enum_partition(struct switchtec_ntb *sndev,
				    u64 *bar_addrs)
{
	struct part_cfg_regs __iomem *part_cfg =
		&sndev->stdev->mmio_part_cfg_all[sndev->peer_partition];
	u32 pff = ioread32(&part_cfg->vep_pff_inst_id) & 0xFF; /// KW_DB
    ///u32 pff = 0; // KW_DB
	// struct pff_csr_regs __iomem *mmio_pff =// KW_DB
		// &sndev->stdev->mmio_pff_csr[pff];// KW_DB
	struct pff_csr_regs __iomem *mmio_pff = &sndev->stdev->mmio_pff_csr[pff];// KW_DB
	const u64 bar_space = 0x1000000000LL;
	u64 bar_addr;
	int bar_cnt = 0;
	int i;

    dev_info(&sndev->stdev->dev, "%s(): Size of (struct part_cfg_regs) = x%lx (Ln %d)\n", __func__, sizeof(struct part_cfg_regs), __LINE__);
    dev_info(&sndev->stdev->dev, "%s(): peer_partition=%d (Ln %d)\n", __func__, sndev->peer_partition, __LINE__);  // KW_DB
    dev_info(&sndev->stdev->dev, " vep_pff        = %d\n", pff); // KW_DB
    dev_info(&sndev->stdev->dev, " status  = %d\n", ioread32(&part_cfg->status)); // KW_DB
    dev_info(&sndev->stdev->dev, " state  = %d\n", ioread32(&part_cfg->state)); // KW_DB
    dev_info(&sndev->stdev->dev, " usp_port_mode  = %d\n", ioread32(&part_cfg->usp_port_mode)); // KW_DB
    dev_info(&sndev->stdev->dev, " usp_pff        = %d\n", ioread32(&part_cfg->usp_pff_inst_id)& 0xFF); // KW_DB
    dev_info(&sndev->stdev->dev, " port_cnt       = %d\n", ioread32(&part_cfg->port_cnt)); // KW_DB

    
	iowrite16(0x6, &mmio_pff->pcicmd);

    /// KW_DB->
    #if 0
    {
        u64 val1= 0xABC0000000LL;
        u64 val2= 0x1000000000LL;
        
        dev_info(&sndev->stdev->dev, "val1 : %llx\n",val1); // KW_DB
        dev_info(&sndev->stdev->dev, "val2 : %llx\n",val2); // KW_DB
        val1 &= ~0xf;
        dev_info(&sndev->stdev->dev, "val1 : %llx\n",val1); 
    }
    #endif
    /// KW_DB<-

    dev_info(&sndev->stdev->dev, "%s(): nr_direct_mw[%d] (Ln %d)\n",
        __func__, sndev->nr_direct_mw, __LINE__); // KW_DB

	for (i = 0; i < ARRAY_SIZE(mmio_pff->pci_bar64); i++) {
		iowrite64(bar_space * i, &mmio_pff->pci_bar64[i]);
        dev_info(&sndev->stdev->dev, "------ %d ------", i); // KW_DB
        dev_info(&sndev->stdev->dev, "Array pci_bar64 : %llx\n",(u64) &mmio_pff->pci_bar64[i]); // KW_DB
		bar_addr = ioread64(&mmio_pff->pci_bar64[i]);
        dev_err(&sndev->stdev->dev, "Raw bar_addr  : %llx\n",bar_addr); // KW_DB
        dev_err(&sndev->stdev->dev, "    bar_space : %llx\n",bar_space); // KW_DB
		bar_addr &= ~0xf;

		dev_dbg(&sndev->stdev->dev,
			"Crosslink BAR%d addr: %llx\n",
			i*2, bar_addr);
		dev_err(&sndev->stdev->dev,"Crosslink BAR%d addr: %llx (%llx) i[%d]\n", i*2, bar_addr, bar_space * i, i); // KW_DB
        
		if (bar_addr != bar_space * i)
			continue;

		bar_addrs[bar_cnt++] = bar_addr;
        dev_err(&sndev->stdev->dev, "Accepted bar_addr : %llx\n",bar_addr); // KW_DB
	}
    ///dev_info(&sndev->stdev->dev, "Array pci_bar64 : %llx\n",(u64) &mmio_pff->pci_bar64[3]); // KW_DB
  
    dev_err(&sndev->stdev->dev, "bar_cnt[%d]\n", bar_cnt); // KW_DB
	return bar_cnt;
}

static int switchtec_ntb_init_crosslink(struct switchtec_ntb *sndev)
{
	int rc;
	int bar = sndev->direct_mw_to_bar[0];
	const int dbmsg_lut_idx = 1;
	const int req_id_lut_idx = 2;
	u64 bar_addrs[6];
	u64 addr;
	int offset;
	int bar_cnt;
    
    dev_info(&sndev->stdev->dev,"%s(): bar[%d] MAX_DIRECT_MW[%ld] (Ln %d)\n",
        __func__, bar, MAX_DIRECT_MW, __LINE__); // KW_DB

	if (!crosslink_is_enabled(sndev))
		return 0;
    // KW_DB->
    {
        struct ntb_info_regs __iomem *inf = sndev->mmio_ntb;
        dev_info(&sndev->stdev->dev, "%s():    xlink_enabled[x%08X] (Ln %d)\n",
            __func__, ioread32(&inf->ntp_info[sndev->peer_partition].xlink_enabled), __LINE__);
        dev_info(&sndev->stdev->dev, "%s():  target_part_low[x%08X] (Ln %d)\n",
            __func__, ioread32(&inf->ntp_info[sndev->peer_partition].target_part_low), __LINE__);  
        dev_info(&sndev->stdev->dev, "%s():  target_part_high[x%08X] (Ln %d)\n",
            __func__, ioread32(&inf->ntp_info[sndev->peer_partition].target_part_high), __LINE__);            
        dev_info(&sndev->stdev->dev, "%s():          reserved[x%08X] (Ln %d)\n",
            __func__, ioread32(&inf->ntp_info[sndev->peer_partition].reserved), __LINE__);              
    }
    // KW_DB<-
    
	dev_info(&sndev->stdev->dev, "Using crosslink configuration\n");
	sndev->ntb.topo = NTB_TOPO_CROSSLINK;

	bar_cnt = crosslink_enum_partition(sndev, bar_addrs);
    dev_info(&sndev->stdev->dev,"%s(): bar_cnt[%d] nr_direct_mw[%d] (Ln %d)\n",
        __func__, bar_cnt, sndev->nr_direct_mw, __LINE__); // KW_DB
    ///#if 0 // KW_DB
	if (bar_cnt < sndev->nr_direct_mw + 1) {
		dev_err(&sndev->stdev->dev,
			"Error enumerating crosslink partition\n");
		return -EINVAL;
	}
    ///#endif // KW_DB

	addr = (bar_addrs[0] + SWITCHTEC_GAS_NTB_OFFSET +
		SWITCHTEC_NTB_REG_DBMSG_OFFSET +
		sizeof(struct ntb_dbmsg_regs) * sndev->peer_partition);

    // SWITCHTEC_GAS_NTB_OFFSET        = 0x10000   - KW_DB
    // SWITCHTEC_NTB_REG_DBMSG_OFFSET  = 0x64000   - KW_DB
    dev_info(&sndev->stdev->dev, "%s(): addr[x%llx] (Ln %d)\n", __func__, addr, __LINE__);  // KW_DB
	offset = addr & (LUT_SIZE - 1);
    dev_info(&sndev->stdev->dev, "%s(): offset[x%x] (Ln %d)\n", __func__, offset, __LINE__);  // KW_DB
	addr -= offset;
    dev_info(&sndev->stdev->dev, "%s(): addr[x%llx] (Ln %d)\n", __func__, addr, __LINE__);  // KW_DB

    dev_info(&sndev->stdev->dev, "%s(): barAdr[x%llx] sz[x%lx] peer_partition[%d] addr[x%llx] (Ln %d)\n", 
        __func__, bar_addrs[0], sizeof(struct ntb_dbmsg_regs), sndev->peer_partition, addr, __LINE__);  // KW_DB
	dev_info(&sndev->stdev->dev, "%s(): Calling config_rsvd_lut_win(%d) (Ln %d)\n", __func__, dbmsg_lut_idx, __LINE__);  // KW_DB
    rc = config_rsvd_lut_win(sndev, sndev->mmio_self_ctrl, dbmsg_lut_idx,
				 sndev->peer_partition, addr);
	if (rc)
		return rc;

	sndev->mmio_xlink_dbmsg_win = pci_iomap_range(sndev->stdev->pdev, bar,
						      dbmsg_lut_idx * LUT_SIZE,
						      LUT_SIZE);
	if (!sndev->mmio_xlink_dbmsg_win) {
        dev_info(&sndev->stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		rc = -ENOMEM;
		return rc;
	}

	sndev->mmio_peer_dbmsg = sndev->mmio_xlink_dbmsg_win + offset;
	sndev->nr_rsvd_luts++;

	addr = (bar_addrs[0] + SWITCHTEC_GAS_NTB_OFFSET +
		SWITCHTEC_NTB_REG_CTRL_OFFSET +
		sizeof(struct ntb_ctrl_regs) * sndev->peer_partition);

    dev_info(&sndev->stdev->dev, "%s(): addr[x%llx] (Ln %d)\n", __func__, addr, __LINE__);  // KW_DB
	offset = addr & (LUT_SIZE - 1);
    dev_info(&sndev->stdev->dev, "%s(): offset[x%x] (Ln %d)\n", __func__, offset, __LINE__);  // KW_DB
	addr -= offset;
    dev_info(&sndev->stdev->dev, "%s(): addr[x%llx] (Ln %d)\n", __func__, addr, __LINE__);  // KW_DB

    dev_info(&sndev->stdev->dev, "%s(): barAdr[x%llx] sz[x%lx] peer_partition[%d] addr[x%llx] (Ln %d)\n", 
        __func__, bar_addrs[0], sizeof(struct ntb_dbmsg_regs), sndev->peer_partition, addr, __LINE__);  // KW_DB
	dev_info(&sndev->stdev->dev, "%s(): Calling config_rsvd_lut_win(%d) (Ln %d)\n", __func__, req_id_lut_idx, __LINE__);  // KW_DB
    rc = config_rsvd_lut_win(sndev, sndev->mmio_self_ctrl,
				 req_id_lut_idx,
				 sndev->peer_partition, addr);
	if (rc)
		return rc;

	sndev->mmio_xlink_ctrl_win = pci_iomap_range(sndev->stdev->pdev, bar,
						     req_id_lut_idx * LUT_SIZE,
						     LUT_SIZE);
	if (!sndev->mmio_xlink_ctrl_win) {
        dev_info(&sndev->stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		rc = -ENOMEM;
		return rc;
	}

	sndev->mmio_xlink_peer_ctrl = sndev->mmio_xlink_ctrl_win + offset;
	sndev->nr_rsvd_luts++;

	rc = crosslink_setup_mws(sndev, dbmsg_lut_idx, req_id_lut_idx,
				 &bar_addrs[1], bar_cnt - 1);
	if (rc)
		return rc;

	return 0;
}

static void switchtec_ntb_deinit_crosslink(struct switchtec_ntb *sndev)
{
	if (sndev->mmio_xlink_dbmsg_win)
		pci_iounmap(sndev->stdev->pdev, sndev->mmio_xlink_dbmsg_win);

	if (sndev->mmio_xlink_ctrl_win)
		pci_iounmap(sndev->stdev->pdev, sndev->mmio_xlink_ctrl_win);
}

static int map_bars(int *map, struct ntb_ctrl_regs __iomem *ctrl)
{
	int i;
	int cnt = 0;

	for (i = 0; i < ARRAY_SIZE(ctrl->bar_entry); i++) {
		u32 r = ioread32(&ctrl->bar_entry[i].ctl);

        printk("%s(): %d: r[x%X] win_size[0x%X] xlate_addr[x%llX] (Ln %d)\n",
            __func__, i, r, ioread32(&ctrl->bar_entry[i].win_size), 
            ioread64(&ctrl->bar_entry[i].xlate_addr),__LINE__); // KW_DB
		if (r & NTB_CTRL_BAR_VALID)
			map[cnt++] = i;
	}

    printk("%s(): cnt[%d] (Ln %d)\n", __func__, cnt, __LINE__); // KW_DB
	return cnt;
}

static void switchtec_ntb_init_mw(struct switchtec_ntb *sndev)
{
	sndev->nr_direct_mw = map_bars(sndev->direct_mw_to_bar,
				       sndev->mmio_self_ctrl);

    ///dev_info(&sndev->stdev->dev, "%s(): nr_direct_mw[%d] (Ln %d)\n",
    ///    __func__, sndev->nr_direct_mw, __LINE__); // KW_DB
	sndev->nr_lut_mw = ioread16(&sndev->mmio_self_ctrl->lut_table_entries);
	sndev->nr_lut_mw = rounddown_pow_of_two(sndev->nr_lut_mw);

	dev_dbg(&sndev->stdev->dev, "MWs: %d direct, %d lut\n",
		sndev->nr_direct_mw, sndev->nr_lut_mw);
	dev_info(&sndev->stdev->dev, "%s(): MWs: %d direct, %d lut\n",
		__func__, sndev->nr_direct_mw, sndev->nr_lut_mw); // KW_DB
        
	sndev->peer_nr_direct_mw = map_bars(sndev->peer_direct_mw_to_bar,
					    sndev->mmio_peer_ctrl);

	sndev->peer_nr_lut_mw =
		ioread16(&sndev->mmio_peer_ctrl->lut_table_entries);
	sndev->peer_nr_lut_mw = rounddown_pow_of_two(sndev->peer_nr_lut_mw);

	dev_dbg(&sndev->stdev->dev, "Peer MWs: %d direct, %d lut\n",
		sndev->peer_nr_direct_mw, sndev->peer_nr_lut_mw);
	dev_info(&sndev->stdev->dev, "%s(): Peer MWs: %d direct, %d lut\n",
		__func__, sndev->peer_nr_direct_mw, sndev->peer_nr_lut_mw);   // KW_DB

}

/*
 * There are 64 doorbells in the switch hardware but this is
 * shared among all partitions. So we must split them in half
 * (32 for each partition). However, the message interrupts are
 * also shared with the top 4 doorbells so we just limit this to
 * 28 doorbells per partition.
 *
 * In crosslink mode, each side has it's own dbmsg register so
 * they can each use all 60 of the available doorbells.
 */
static void switchtec_ntb_init_db(struct switchtec_ntb *sndev)
{
	sndev->db_mask = 0x0FFFFFFFFFFFFFFFULL;

	if (sndev->mmio_peer_dbmsg != sndev->mmio_self_dbmsg) {
		sndev->db_shift = 0;
		sndev->db_peer_shift = 0;
		sndev->db_valid_mask = sndev->db_mask;
	} else if (sndev->self_partition < sndev->peer_partition) {
		sndev->db_shift = 0;
		sndev->db_peer_shift = 32;
		sndev->db_valid_mask = 0x0FFFFFFF;
	} else {
		sndev->db_shift = 32;
		sndev->db_peer_shift = 0;
		sndev->db_valid_mask = 0x0FFFFFFF;
	}

	iowrite64(~sndev->db_mask, &sndev->mmio_self_dbmsg->idb_mask);
	iowrite64(sndev->db_valid_mask << sndev->db_peer_shift,
		  &sndev->mmio_peer_dbmsg->odb_mask);

	dev_dbg(&sndev->stdev->dev, "dbs: shift %d/%d, mask %016llx\n",
		sndev->db_shift, sndev->db_peer_shift, sndev->db_valid_mask);
}

static void switchtec_ntb_init_msgs(struct switchtec_ntb *sndev)
{
	int i;
	u32 msg_map = 0;

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_dbmsg->imsg); i++) {
		int m = i | sndev->peer_partition << 2;

		msg_map |= m << i * 8;
	}

	iowrite32(msg_map, &sndev->mmio_self_dbmsg->msg_map);

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_dbmsg->imsg); i++)
		iowrite64(NTB_DBMSG_IMSG_STATUS | NTB_DBMSG_IMSG_MASK,
			  &sndev->mmio_self_dbmsg->imsg[i]);
}

static int
switchtec_ntb_init_req_id_table(struct switchtec_ntb *sndev)
{
	int req_id;
	int rc;

	/*
	 * Root Complex Requester ID (which is 0:00.0)
	 */
    dev_info(&sndev->stdev->dev, "%s(): Adding req_id[0] (Ln %d)\n", __func__, __LINE__);  // KW_DB
	rc = add_req_id(sndev, sndev->mmio_self_ctrl, 0);
	if (rc)
		return rc;

	/*
	 * Host Bridge Requester ID (as read from the mmap address)
	 */
	req_id = ioread16(&sndev->mmio_ntb->requester_id);
    dev_info(&sndev->stdev->dev, "%s(): Adding req_id[x%x] (Ln %d)\n", __func__, req_id, __LINE__);  // KW_DB
	return add_req_id(sndev, sndev->mmio_self_ctrl, req_id);
}

static void switchtec_ntb_init_shared(struct switchtec_ntb *sndev)
{
	int i;

    dev_info(&sndev->stdev->dev, "%s(): partition[%d] (Ln %d)\n",
        __func__, sndev->stdev->partition, __LINE__);  // KW_DB
	memset(sndev->self_shared, 0, LUT_SIZE);
	sndev->self_shared->magic = SWITCHTEC_NTB_MAGIC;
	sndev->self_shared->partition_id = sndev->stdev->partition;

	for (i = 0; i < sndev->nr_direct_mw; i++) {
		int bar = sndev->direct_mw_to_bar[i];
		resource_size_t sz = pci_resource_len(sndev->stdev->pdev, bar);

		if (i == 0)
			sz = min_t(resource_size_t, sz,
				   LUT_SIZE * sndev->nr_lut_mw);

		sndev->self_shared->mw_sizes[i] = sz;
        dev_info(&sndev->stdev->dev, "%s(): direct_mw[%d] sz[%lld] (Ln %d)\n",
            __func__, i, sz, __LINE__);  // KW_DB
	}

	for (i = 0; i < sndev->nr_lut_mw; i++) {
		int idx = sndev->nr_direct_mw + i;

        // KW_DB_NEW->
		if (idx >= MAX_MWS)	// KW_DB - Apparent, it goes out of range in our case.
		{
			dev_err(&sndev->stdev->dev, "%s() idx[%d] out-of-range [%d] (Ln %d)\n",
				__func__, idx, MAX_MWS, __LINE__); // KW_DB
			break;				// KW_DB
		}
        // KW_DB_NEW<-
        
		sndev->self_shared->mw_sizes[idx] = LUT_SIZE;
        ///dev_info(&sndev->stdev->dev, "%s(): lut_mw[%d] sz[%d] (Ln %d)\n",
        ///    __func__, idx, LUT_SIZE, __LINE__);  // KW_DB        
	}
    
    dev_info(&sndev->stdev->dev, "%s(): Leaving (Ln %d)\n", __func__, __LINE__);  // KW_DB
}

static int switchtec_ntb_init_shared_mw(struct switchtec_ntb *sndev)
{
	int self_bar = sndev->direct_mw_to_bar[0];
	int rc;

	sndev->nr_rsvd_luts++;
	sndev->self_shared = dma_alloc_coherent(&sndev->stdev->pdev->dev,
						LUT_SIZE,
						&sndev->self_shared_dma,
						GFP_KERNEL);
	if (!sndev->self_shared) {
		dev_err(&sndev->stdev->dev,
			"unable to allocate memory for shared mw\n");
        dev_info(&sndev->stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -ENOMEM;
	}

    ///dev_info(&sndev->stdev->dev, "    %s: self_shared[0x%X] (Ln %d)\n",
    ///    __func__, sndev->self_shared, __LINE__);  // KW_DB
    
    dev_info(&sndev->stdev->dev, "%s(): Calling switchtec_ntb_init_shared() (Ln %d)\n", __func__, __LINE__);  // KW_DB
	switchtec_ntb_init_shared(sndev);

    dev_info(&sndev->stdev->dev, "%s(): Calling config_rsvd_lut_win(0) (Ln %d)\n", __func__, __LINE__);  // KW_DB
	rc = config_rsvd_lut_win(sndev, sndev->mmio_peer_ctrl, 0,
				 sndev->self_partition,
				 sndev->self_shared_dma);
	if (rc)
		goto unalloc_and_exit;

    dev_info(&sndev->stdev->dev, "%s(): Calling pci_iomap() self_bar[%d] (Ln %d)\n", __func__, self_bar, __LINE__);  // KW_DB
	sndev->peer_shared = pci_iomap(sndev->stdev->pdev, self_bar, LUT_SIZE);
	if (!sndev->peer_shared) {
        dev_info(&sndev->stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		rc = -ENOMEM;
		goto unalloc_and_exit;
	}

	dev_dbg(&sndev->stdev->dev, "Shared MW Ready\n");
	return 0;

unalloc_and_exit:
	dma_free_coherent(&sndev->stdev->pdev->dev, LUT_SIZE,
			  sndev->self_shared, sndev->self_shared_dma);

	return rc;
}

#if (SK_HYNIX == 1)
#if 0   // Not used anymore.
/**
 * Initialize special purpose memory windows.
 */
static int sk_ntb_init_sp_mw(struct switchtec_ntb *sndev)
{
	// int self_bar = sndev->direct_mw_to_bar[0];
	int rc;    
    
	sndev->nr_rsvd_luts++;
	sndev->sp_mw_01_shared = dma_alloc_coherent(&sndev->stdev->pdev->dev,
						LUT_SIZE,
						&sndev->sp_mw_01_shared_dma,
						GFP_KERNEL);
	if (!sndev->sp_mw_01_shared) {
		dev_err(&sndev->stdev->dev,
			"unable to allocate memory for shared mw\n");
        dev_info(&sndev->stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -ENOMEM;
	}


    dev_info(&sndev->stdev->dev, "%s(): Calling config_rsvd_lut_win(3) (Ln %d)\n", __func__, __LINE__);  // KW_DB
	rc = config_rsvd_lut_win(sndev, sndev->mmio_peer_ctrl, 3,
				 sndev->self_partition,
				 sndev->sp_mw_01_shared_dma);
	if (rc)
		goto unalloc_and_exit;
    
    // dev_info(&sndev->stdev->dev, "%s(): Calling pci_iomap() self_bar[%d] (Ln %d)\n", __func__, self_bar, __LINE__);  // KW_DB
	// sndev->peer_sp_mw_01_shared = pci_iomap(sndev->stdev->pdev, self_bar, LUT_SIZE);
	// if (!sndev->peer_sp_mw_01_shared) {
        // dev_info(&sndev->stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		// rc = -ENOMEM;
		// goto unalloc_and_exit;
	// }
    sndev->peer_sp_mw_01_shared = (void __iomem *) sndev->peer_shared;

	dev_dbg(&sndev->stdev->dev, "SP MW a Ready\n");
    
    
	return 0;

unalloc_and_exit:
	dma_free_coherent(&sndev->stdev->pdev->dev, LUT_SIZE,
			  sndev->sp_mw_01_shared, sndev->sp_mw_01_shared_dma);  

    return rc;
}
#endif  /* 0 */
#endif  /* SK_HYNIX */

static void switchtec_ntb_deinit_shared_mw(struct switchtec_ntb *sndev)
{
	if (sndev->peer_shared)
		pci_iounmap(sndev->stdev->pdev, sndev->peer_shared);

#if (SK_HYNIX == 1)
	if (sndev->sp_mw_01_shared)
		dma_free_coherent(&sndev->stdev->pdev->dev, LUT_SIZE,
				  sndev->sp_mw_01_shared,
				  sndev->sp_mw_01_shared_dma);
	sndev->nr_rsvd_luts--;    
#endif
    
	if (sndev->self_shared)
		dma_free_coherent(&sndev->stdev->pdev->dev, LUT_SIZE,
				  sndev->self_shared,
				  sndev->self_shared_dma);
	sndev->nr_rsvd_luts--;
}

static irqreturn_t switchtec_ntb_doorbell_isr(int irq, void *dev)
{
	struct switchtec_ntb *sndev = dev;

	dev_dbg(&sndev->stdev->dev, "doorbell\n");

	ntb_db_event(&sndev->ntb, 0);

	return IRQ_HANDLED;
}

static irqreturn_t switchtec_ntb_message_isr(int irq, void *dev)
{
	int i;
	struct switchtec_ntb *sndev = dev;

	for (i = 0; i < ARRAY_SIZE(sndev->mmio_self_dbmsg->imsg); i++) {
		u64 msg = ioread64(&sndev->mmio_self_dbmsg->imsg[i]);

		if (msg & NTB_DBMSG_IMSG_STATUS) {
			dev_dbg(&sndev->stdev->dev, "message: %d %08x\n",
				i, (u32)msg);
			iowrite8(1, &sndev->mmio_self_dbmsg->imsg[i].status);

			if (i == LINK_MESSAGE)
				switchtec_ntb_check_link(sndev, msg);
		}
	}

	return IRQ_HANDLED;
}

static int switchtec_ntb_init_db_msg_irq(struct switchtec_ntb *sndev)
{
	int i;
	int rc;
	int doorbell_irq = 0;
	int message_irq = 0;
	int event_irq;
	int idb_vecs = sizeof(sndev->mmio_self_dbmsg->idb_vec_map);

	event_irq = ioread32(&sndev->stdev->mmio_part_cfg->vep_vector_number);

	while (doorbell_irq == event_irq)
		doorbell_irq++;
	while (message_irq == doorbell_irq ||
	       message_irq == event_irq)
		message_irq++;

	dev_dbg(&sndev->stdev->dev, "irqs - event: %d, db: %d, msgs: %d\n",
		event_irq, doorbell_irq, message_irq);

	for (i = 0; i < idb_vecs - 4; i++)
		iowrite8(doorbell_irq,
			 &sndev->mmio_self_dbmsg->idb_vec_map[i]);

	for (; i < idb_vecs; i++)
		iowrite8(message_irq,
			 &sndev->mmio_self_dbmsg->idb_vec_map[i]);

	sndev->doorbell_irq = pci_irq_vector(sndev->stdev->pdev, doorbell_irq);
	sndev->message_irq = pci_irq_vector(sndev->stdev->pdev, message_irq);

	rc = request_irq(sndev->doorbell_irq,
			 switchtec_ntb_doorbell_isr, 0,
			 "switchtec_ntb_doorbell", sndev);
	if (rc)
		return rc;

	rc = request_irq(sndev->message_irq,
			 switchtec_ntb_message_isr, 0,
			 "switchtec_ntb_message", sndev);
	if (rc) {
		free_irq(sndev->doorbell_irq, sndev);
		return rc;
	}

	return 0;
}

static void switchtec_ntb_deinit_db_msg_irq(struct switchtec_ntb *sndev)
{
	free_irq(sndev->doorbell_irq, sndev);
	free_irq(sndev->message_irq, sndev);
}

static int switchtec_ntb_reinit_peer(struct switchtec_ntb *sndev)
{
	int rc;

	if (crosslink_is_enabled(sndev))
		return 0;

	dev_info(&sndev->stdev->dev, "reinitialize shared memory window\n");

    dev_info(&sndev->stdev->dev, "%s(): Calling config_rsvd_lut_win(0) (Ln %d)\n", __func__, __LINE__);  // KW_DB
	rc = config_rsvd_lut_win(sndev, sndev->mmio_peer_ctrl, 0,
				 sndev->self_partition,
				 sndev->self_shared_dma);
	return rc;
}

static ssize_t add_requester_id_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct ntb_dev *ntb = container_of(dev, struct ntb_dev, dev);
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	int req_id;
	int bus, device, func;
	int rc;

	if (sscanf(buf, "%x:%x.%x", &bus, &device, &func) != 3)
		return -EINVAL;

	req_id = PCI_DEVID(bus, PCI_DEVFN(device, func));
    dev_info(&sndev->stdev->dev, "%s(): Adding req_id[x%x] (Ln %d)\n", __func__, req_id, __LINE__);  // KW_DB
	rc = add_req_id(sndev, sndev->mmio_self_ctrl, req_id);
	if (rc)
		return rc;

	if (crosslink_is_enabled(sndev)) {
		rc = crosslink_setup_req_ids(sndev,
					     sndev->mmio_xlink_peer_ctrl);
		if (rc)
			return rc;
	}

	return count;
}
static DEVICE_ATTR_WO(add_requester_id);

static ssize_t del_requester_id_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct ntb_dev *ntb = container_of(dev, struct ntb_dev, dev);
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	int req_id;
	int bus, device, func;
	int rc;

	if (sscanf(buf, "%x:%x.%x", &bus, &device, &func) != 3)
		return -EINVAL;

	req_id = PCI_DEVID(bus, PCI_DEVFN(device, func));
	rc = del_req_id(sndev, sndev->mmio_self_ctrl, req_id);
	if (rc)
		return rc;

	if (crosslink_is_enabled(sndev)) {
		rc = crosslink_setup_req_ids(sndev,
					     sndev->mmio_xlink_peer_ctrl);
		if (rc)
			return rc;
	}

	return count;
}
static DEVICE_ATTR_WO(del_requester_id);

static ssize_t requester_ids_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ntb_dev *ntb = container_of(dev, struct ntb_dev, dev);
	struct switchtec_ntb *sndev = ntb_sndev(ntb);
	int i;
	int table_size;
	char req_id_str[32];
	u32 req_id;
	ssize_t n = 0;

	table_size = ioread16(&sndev->mmio_self_ctrl->req_id_table_size);

	for (i = 0; i < table_size; i++) {
		req_id = ioread32(&sndev->mmio_self_ctrl->req_id_table[i]);

		if (req_id & NTB_CTRL_REQ_ID_EN) {
			req_id >>= 16;
			n += sprintf(req_id_str, "%d\t%02X:%02X.%X\n", i,
				     PCI_BUS_NUM(req_id), PCI_SLOT(req_id),
				     PCI_FUNC(req_id));
			strcat(buf, req_id_str);
		}
	}

	return n;
}
static DEVICE_ATTR_RO(requester_ids);

static struct attribute *switchtec_ntb_device_attrs[] = {
	&dev_attr_add_requester_id.attr,
	&dev_attr_del_requester_id.attr,
	&dev_attr_requester_ids.attr,
	NULL,
};

static const struct attribute_group switchtec_ntb_device_group = {
	.attrs = switchtec_ntb_device_attrs,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,5,0)    // KW_DB
static int switchtec_ntb_add(struct device *dev,
			     struct class_interface *class_intf)
#else
static int switchtec_ntb_add(struct device *dev)
#endif
{
	struct switchtec_dev *stdev = to_stdev(dev);
	struct switchtec_ntb *sndev;
	int rc;

    ///dev_err(dev, "%s(): Entered (Ln %d)\n", __func__, __LINE__);  // KW_DB
    
	stdev->sndev = NULL;

	if (stdev->pdev->class != (PCI_CLASS_BRIDGE_OTHER << 8))
    {
        dev_err(dev, "%s(): -ENODEV (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -ENODEV;
    }

    ///dev_err(dev, "%s(): Here (Ln %d)\n", __func__, __LINE__);  // KW_DB
	sndev = kzalloc_node(sizeof(*sndev), GFP_KERNEL, dev_to_node(dev));
	if (!sndev)
    {   // KW_DB
        dev_info(&stdev->dev, "    %s: -ENOMEM (Ln %d)\n", __func__, __LINE__);  // KW_DB
		return -ENOMEM;
    }   // KW_DB

    ///dev_err(dev, "%s(): Here (Ln %d)\n", __func__, __LINE__);  // KW_DB
	sndev->stdev = stdev;
	rc = switchtec_ntb_init_sndev(sndev);
	if (rc)
    {  // KW_DB
        dev_err(dev, "failed 1\n");  // KW_DB
		goto free_and_exit;
    }  // KW_DB

    ///dev_info(dev, "%s(): Calling switchtec_ntb_init_mw() (Ln %d)\n", __func__, __LINE__);  // KW_DB
	switchtec_ntb_init_mw(sndev);
    
    ///dev_err(dev, "%s(): Here (Ln %d)\n", __func__, __LINE__);  // KW_DB
	rc = switchtec_ntb_init_req_id_table(sndev);
	if (rc)
    {  // KW_DB
        dev_err(dev, "failed 2\n");  // KW_DB
		goto free_and_exit;
    }  // KW_DB

    ///dev_info(dev, "%s(): Calling switchtec_ntb_init_crosslink() (Ln %d)\n", __func__, __LINE__);  // KW_DB
	rc = switchtec_ntb_init_crosslink(sndev);
	if (rc)
    {  // KW_DB
        dev_err(dev, "Failed switchtec_ntb_init_crosslink()\n");  // KW_DB
		goto free_and_exit;
    }  // KW_DB

	switchtec_ntb_init_db(sndev);
	switchtec_ntb_init_msgs(sndev);

	rc = switchtec_ntb_init_shared_mw(sndev);
	if (rc)
		goto deinit_crosslink;

	rc = switchtec_ntb_init_db_msg_irq(sndev);
	if (rc)
		goto deinit_shared_and_exit;

#if (SK_HYNIX == 1)
  #if 0 // Not reserving LUT here anymore because it cannot be accessed
        // by NTB client. Should do it in NTB client instead.
    rc = sk_ntb_init_sp_mw(sndev);
	if (rc)
		goto deinit_and_exit;
  #endif
#endif
    
	/*
	 * If this host crashed, the other host may think the link is
	 * still up. Tell them to force it down (it will go back up
	 * once we register the ntb device).
	 */
	switchtec_ntb_send_msg(sndev, LINK_MESSAGE, MSG_LINK_FORCE_DOWN);

	rc = ntb_register_device(&sndev->ntb);
	if (rc)
		goto deinit_and_exit;

	rc = sysfs_create_group(&sndev->ntb.dev.kobj,
				&switchtec_ntb_device_group);
	if (rc)
		goto deinit_and_exit;

	stdev->sndev = sndev;
	stdev->link_notifier = switchtec_ntb_link_notification;
	dev_info(dev, "NTB device registered\n");

	return 0;

deinit_and_exit:
	switchtec_ntb_deinit_db_msg_irq(sndev);
deinit_shared_and_exit:
	switchtec_ntb_deinit_shared_mw(sndev);
deinit_crosslink:
	switchtec_ntb_deinit_crosslink(sndev);
free_and_exit:
	kfree(sndev);
	// dev_err(dev, "failed to register ntb device: %d\n", rc); // KW_DB
    dev_err(dev, "failed to register NTB device: %d\n", rc);    // KW_DB
	return rc;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(6,5,0)    // KW_DB
static void switchtec_ntb_remove(struct device *dev,
				 struct class_interface *class_intf)
#else
static void switchtec_ntb_remove(struct device *dev)
#endif
{
	struct switchtec_dev *stdev = to_stdev(dev);
	struct switchtec_ntb *sndev = stdev->sndev;

	if (!sndev)
		return;

	flush_scheduled_work();  /// KW_DB_20240815 - 6.8.0-40-generic not supporting it

	stdev->link_notifier = NULL;
	stdev->sndev = NULL;
	sysfs_remove_group(&sndev->ntb.dev.kobj, &switchtec_ntb_device_group);
	ntb_unregister_device(&sndev->ntb);
	switchtec_ntb_deinit_db_msg_irq(sndev);
	switchtec_ntb_deinit_shared_mw(sndev);
	switchtec_ntb_deinit_crosslink(sndev);
	kfree(sndev);
	dev_info(dev, "ntb device unregistered\n");
}

static struct class_interface switchtec_interface  = {
	.add_dev = switchtec_ntb_add,
	.remove_dev = switchtec_ntb_remove,
};

static int __init switchtec_ntb_init(void)
{
	switchtec_interface.class = switchtec_class;
	return class_interface_register(&switchtec_interface);
}
module_init(switchtec_ntb_init);

static void __exit switchtec_ntb_exit(void)
{
	class_interface_unregister(&switchtec_interface);
}
module_exit(switchtec_ntb_exit);
