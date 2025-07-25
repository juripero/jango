// SPDX-License-Identifier: GPL-2.0
/*
 * NVM Express device driver
 * Copyright (c) 2011-2014, Intel Corporation.
 */

#include <linux/acpi.h>
#include <linux/async.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/blk-mq-pci.h>
#include <linux/blk-integrity.h>
#include <linux/delay.h>   // KW_DB
#include <linux/dmi.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kstrtox.h>
#include <linux/memremap.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/once.h>
#include <linux/pci.h>
#include <linux/suspend.h>
#include <linux/t10-pi.h>
#include <linux/types.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/io-64-nonatomic-hi-lo.h>
#include <linux/sed-opal.h>
#include <linux/pci-p2pdma.h>
///#include <linux/ntb.h>  // KW_DB

#include "../../Microsemi/switchtec-kernel/ntb_nvme_inf.h" // KW_DB

#include "trace.h"
#include "nvme.h"

// KW_DB->

#define DYNAMIC_WIDX    (1)
#define USE_MW_PRP      (1) // KW_DB_20241211 - Use a single memory window for all PRP mapping.
                            //                  If enabled, should also enable for NTB driver.
#define USE_MW_SQ_CQ    (1) // KW_DB_20250124 - Use a single memory window for all SQ/CQ mapping.

#if 1
#define SQ_SIZE(q)	(((q)->sq_size_new)? ((q)->sq_size_new) : ((q)->q_depth << (q)->sqes))
#define CQ_SIZE(q)	(((q)->cq_size_new)? ((q)->cq_size_new) : ((q)->q_depth * sizeof(struct nvme_completion)))
#define USE_DBUF    (0)
#else
// KW_DB<-
#define SQ_SIZE(q)	((q)->q_depth << (q)->sqes)
#define CQ_SIZE(q)	((q)->q_depth * sizeof(struct nvme_completion))
#endif  // KW_DB

#define SGES_PER_PAGE	(NVME_CTRL_PAGE_SIZE / sizeof(struct nvme_sgl_desc))

/*
 * These can be higher, but we need to ensure that any command doesn't
 * require an sg allocation that needs more than a page of data.
 */
#define NVME_MAX_KB_SZ	8192
#define NVME_MAX_SEGS	128
#define NVME_MAX_NR_ALLOCATIONS	5

static int use_threaded_interrupts;
module_param(use_threaded_interrupts, int, 0444);

static bool use_cmb_sqes = true;
module_param(use_cmb_sqes, bool, 0444);
MODULE_PARM_DESC(use_cmb_sqes, "use controller's memory buffer for I/O SQes");

static unsigned int max_host_mem_size_mb = 128;
module_param(max_host_mem_size_mb, uint, 0444);
MODULE_PARM_DESC(max_host_mem_size_mb,
	"Maximum Host Memory Buffer (HMB) size per controller (in MiB)");

static unsigned int sgl_threshold = SZ_32K;
module_param(sgl_threshold, uint, 0644);
MODULE_PARM_DESC(sgl_threshold,
		"Use SGLs when average request segment size is larger or equal to "
		"this size. Use 0 to disable SGLs.");

#define NVME_PCI_MIN_QUEUE_SIZE 2
#define NVME_PCI_MAX_QUEUE_SIZE 4095
static int io_queue_depth_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops io_queue_depth_ops = {
	.set = io_queue_depth_set,
	.get = param_get_uint,
};

static unsigned int io_queue_depth = 1024;
module_param_cb(io_queue_depth, &io_queue_depth_ops, &io_queue_depth, 0644);
MODULE_PARM_DESC(io_queue_depth, "set io queue depth, should >= 2 and < 4096");

static int io_queue_count_set(const char *val, const struct kernel_param *kp)
{
	unsigned int n;
	int ret;

	ret = kstrtouint(val, 10, &n);
	if (ret != 0 || n > num_possible_cpus())
		return -EINVAL;
	return param_set_uint(val, kp);
}

static const struct kernel_param_ops io_queue_count_ops = {
	.set = io_queue_count_set,
	.get = param_get_uint,
};

static unsigned int write_queues;
module_param_cb(write_queues, &io_queue_count_ops, &write_queues, 0644);
MODULE_PARM_DESC(write_queues,
	"Number of queues to use for writes. If not set, reads and writes "
	"will share a queue set.");

// static unsigned int poll_queues;     // KW_DB_20241112
static unsigned int poll_queues = 1;    // KW_DB_20241112
module_param_cb(poll_queues, &io_queue_count_ops, &poll_queues, 0644);
MODULE_PARM_DESC(poll_queues, "Number of queues to use for polled IO.");

static bool noacpi;
module_param(noacpi, bool, 0444);
MODULE_PARM_DESC(noacpi, "disable acpi bios quirks");

struct nvme_dev;
struct nvme_queue;

static void nvme_dev_disable(struct nvme_dev *dev, bool shutdown);
static void nvme_delete_io_queues(struct nvme_dev *dev);
static void nvme_update_attrs(struct nvme_dev *dev);
static u32 my_round_down(struct nvme_dev *dev, u32 addr);   // KW_DB_20240627

struct ntb_dev *ntb_temp_holder;    // KW_DB - Temperory holder
/*
 * Represents an NVM Express device.  Each nvme_dev is a PCI function.
 */
struct nvme_dev {
    ///struct ntb_dev *ntb;    // KW_DB
	struct nvme_queue *queues;
	struct blk_mq_tag_set tagset;
	struct blk_mq_tag_set admin_tagset;
	u32 __iomem *dbs;
	struct device *dev;
	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	unsigned online_queues;
	unsigned max_qid;
	unsigned io_queues[HCTX_MAX_TYPES];
	unsigned int num_vecs;
	u32 q_depth;
	int io_sqes;
	u32 db_stride;
	void __iomem *bar;
	unsigned long bar_mapped_size;
	struct mutex shutdown_lock;
	bool subsystem;
	u64 cmb_size;
	bool cmb_use_sqes;
	u32 cmbsz;
	u32 cmbloc;
	struct nvme_ctrl ctrl;
	u32 last_ps;
	bool hmb;

	mempool_t *iod_mempool;

	/* shadow doorbell buffer support: */
	__le32 *dbbuf_dbs;
	dma_addr_t dbbuf_dbs_dma_addr;
	__le32 *dbbuf_eis;
	dma_addr_t dbbuf_eis_dma_addr;

	/* host memory buffer support: */
	u64 host_mem_size;
	u32 nr_host_mem_descs;
	dma_addr_t host_mem_descs_dma;
	struct nvme_host_mem_buf_desc *host_mem_descs;
	void **host_mem_desc_bufs;
	unsigned int nr_allocated_queues;
	unsigned int nr_write_queues;
	unsigned int nr_poll_queues;
    unsigned int nr_irq_queues; // KW_DB_20250116 - # Keep track of # of queues that use irq, (including admin Q).

    // KW_DB->
    u32         dnb_addr_lo;
    u32         dnb_addr_hi;
    dma_addr_t  dnb_addr;
    // KW_DB_20241211->
    u32         mw0_addr_lo;
    u32         mw0_addr_hi;
    dma_addr_t  mw0_addr;
    // KW_DB_20241211<-
    u32         lut_xlat_align;
    u32         lut_size_align;
    u32         lut_size_max;
#if (DYNAMIC_WIDX)
    u32         free_widx;
    u16         msix_widx[33];          // Keep track of widx allocated for queues.
    u32         msix_vector_ldw[33];    // Keep track of MSIX vector lower DW of queues.
    u32         msix_vector_udw[33];    // Keep track of MSIX vector upper DW of queues.
#endif
    // KW_DB<-
};

// KW_DB->
static void read_msix_cap(struct nvme_dev *dev, const char* func, const int line);


#if (DYNAMIC_WIDX)
/**
 */
int getFreeWidx(struct nvme_dev * dev, u32 *free_widx)
{
    // TODO: Revise WIDX allocation. Should be propagated to Switchtec level.
    if (dev->free_widx > MAX_SWITCHTEC_WIDX)
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
        return -ENOMEM;
    }

    *free_widx = dev->free_widx ;
    dev->free_widx++;

    return 0;
}
#endif
// KW_DB<-

static int io_queue_depth_set(const char *val, const struct kernel_param *kp)
{
	return param_set_uint_minmax(val, kp, NVME_PCI_MIN_QUEUE_SIZE,
			NVME_PCI_MAX_QUEUE_SIZE);
}

static inline unsigned int sq_idx(unsigned int qid, u32 stride)
{
	return qid * 2 * stride;
}

static inline unsigned int cq_idx(unsigned int qid, u32 stride)
{
	return (qid * 2 + 1) * stride;
}

static inline struct nvme_dev *to_nvme_dev(struct nvme_ctrl *ctrl)
{
	return container_of(ctrl, struct nvme_dev, ctrl);
}

/*
 * An NVM Express queue.  Each device has at least two (one for admin
 * commands and one for I/O commands).
 */
struct nvme_queue {
	struct nvme_dev *dev;
	spinlock_t sq_lock;
	void *sq_cmds;
	 /* only used for poll queues: */
	spinlock_t cq_poll_lock ____cacheline_aligned_in_smp;
	struct nvme_completion *cqes;
	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	u32 __iomem *q_db;
	u32 q_depth;
    // KW_DB->
    size_t sq_size_orig;
    size_t sq_size_new;
    size_t cq_size_orig;
    size_t cq_size_new;
#if (USE_DBUF)
    /* For Identify Ctrl Cmd experiment */
    void      * dBuf;
    dma_addr_t  dBufDmaAddr;
#endif
    // KW_DB<-
	u16 cq_vector;
	u16 sq_tail;
	u16 last_sq_tail;
	u16 cq_head;
	u16 qid;
	u8 cq_phase;
	u8 sqes;
	unsigned long flags;
#define NVMEQ_ENABLED		0
#define NVMEQ_SQ_CMB		1
#define NVMEQ_DELETE_ERROR	2
#define NVMEQ_POLLED		3
	__le32 *dbbuf_sq_db;
	__le32 *dbbuf_cq_db;
	__le32 *dbbuf_sq_ei;
	__le32 *dbbuf_cq_ei;
	struct completion delete_done;
};

union nvme_descriptor {
	struct nvme_sgl_desc	*sg_list;
	__le64			*prp_list;
};

/*
 * The nvme_iod describes the data in an I/O.
 *
 * The sg pointer contains the list of PRP/SGL chunk allocations in addition
 * to the actual struct scatterlist.
 */
struct nvme_iod {
	struct nvme_request req;
	struct nvme_command cmd;
	bool aborted;
	s8 nr_allocations;	/* PRP list pool allocations. 0 means small
				   pool in use */
	unsigned int dma_len;	/* length of single DMA segment mapping */
	dma_addr_t first_dma;
	dma_addr_t meta_dma;
	struct sg_table sgt;
	union nvme_descriptor list[NVME_MAX_NR_ALLOCATIONS];
};

static inline unsigned int nvme_dbbuf_size(struct nvme_dev *dev)
{
	return dev->nr_allocated_queues * 8 * dev->db_stride;
}

static void nvme_dbbuf_dma_alloc(struct nvme_dev *dev)
{
	unsigned int mem_size = nvme_dbbuf_size(dev);

	if (!(dev->ctrl.oacs & NVME_CTRL_OACS_DBBUF_SUPP))
		return;

	if (dev->dbbuf_dbs) {
		/*
		 * Clear the dbbuf memory so the driver doesn't observe stale
		 * values from the previous instantiation.
		 */
		memset(dev->dbbuf_dbs, 0, mem_size);
		memset(dev->dbbuf_eis, 0, mem_size);
		return;
	}

	dev->dbbuf_dbs = dma_alloc_coherent(dev->dev, mem_size,
					    &dev->dbbuf_dbs_dma_addr,
					    GFP_KERNEL);
	if (!dev->dbbuf_dbs)
		goto fail;
	dev->dbbuf_eis = dma_alloc_coherent(dev->dev, mem_size,
					    &dev->dbbuf_eis_dma_addr,
					    GFP_KERNEL);
	if (!dev->dbbuf_eis)
		goto fail_free_dbbuf_dbs;
	return;

fail_free_dbbuf_dbs:
	dma_free_coherent(dev->dev, mem_size, dev->dbbuf_dbs,
			  dev->dbbuf_dbs_dma_addr);
	dev->dbbuf_dbs = NULL;
fail:
	dev_warn(dev->dev, "unable to allocate dma for dbbuf\n");
}

static void nvme_dbbuf_dma_free(struct nvme_dev *dev)
{
	unsigned int mem_size = nvme_dbbuf_size(dev);

	if (dev->dbbuf_dbs) {
		dma_free_coherent(dev->dev, mem_size,
				  dev->dbbuf_dbs, dev->dbbuf_dbs_dma_addr);
		dev->dbbuf_dbs = NULL;
	}
	if (dev->dbbuf_eis) {
		dma_free_coherent(dev->dev, mem_size,
				  dev->dbbuf_eis, dev->dbbuf_eis_dma_addr);
		dev->dbbuf_eis = NULL;
	}
}

static void nvme_dbbuf_init(struct nvme_dev *dev,
			    struct nvme_queue *nvmeq, int qid)
{
	if (!dev->dbbuf_dbs || !qid)
		return;

	nvmeq->dbbuf_sq_db = &dev->dbbuf_dbs[sq_idx(qid, dev->db_stride)];
	nvmeq->dbbuf_cq_db = &dev->dbbuf_dbs[cq_idx(qid, dev->db_stride)];
	nvmeq->dbbuf_sq_ei = &dev->dbbuf_eis[sq_idx(qid, dev->db_stride)];
	nvmeq->dbbuf_cq_ei = &dev->dbbuf_eis[cq_idx(qid, dev->db_stride)];
}

static void nvme_dbbuf_free(struct nvme_queue *nvmeq)
{
	if (!nvmeq->qid)
		return;

	nvmeq->dbbuf_sq_db = NULL;
	nvmeq->dbbuf_cq_db = NULL;
	nvmeq->dbbuf_sq_ei = NULL;
	nvmeq->dbbuf_cq_ei = NULL;
}

static void nvme_dbbuf_set(struct nvme_dev *dev)
{
	struct nvme_command c = { };
	unsigned int i;

	if (!dev->dbbuf_dbs)
		return;

	c.dbbuf.opcode = nvme_admin_dbbuf;
#if (USE_MW_PRP)    // KW_DB_20250224
	c.dbbuf.prp1 = cpu_to_le64(dev->mw0_addr + dev->dbbuf_dbs_dma_addr);
	c.dbbuf.prp2 = cpu_to_le64(dev->mw0_addr + dev->dbbuf_eis_dma_addr);
#else
	c.dbbuf.prp1 = cpu_to_le64(dev->dbbuf_dbs_dma_addr);
	c.dbbuf.prp2 = cpu_to_le64(dev->dbbuf_eis_dma_addr);
#endif

	if (nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0)) {
		dev_warn(dev->ctrl.device, "unable to set dbbuf\n");
		/* Free memory and continue on */
		nvme_dbbuf_dma_free(dev);

		for (i = 1; i <= dev->online_queues; i++)
			nvme_dbbuf_free(&dev->queues[i]);
	}
}

static inline int nvme_dbbuf_need_event(u16 event_idx, u16 new_idx, u16 old)
{
	return (u16)(new_idx - event_idx - 1) < (u16)(new_idx - old);
}

/* Update dbbuf and return true if an MMIO is required */
static bool nvme_dbbuf_update_and_check_event(u16 value, __le32 *dbbuf_db,
					      volatile __le32 *dbbuf_ei)
{
	if (dbbuf_db) {
		u16 old_value, event_idx;

		/*
		 * Ensure that the queue is written before updating
		 * the doorbell in memory
		 */
		wmb();

		old_value = le32_to_cpu(*dbbuf_db);
		*dbbuf_db = cpu_to_le32(value);

		/*
		 * Ensure that the doorbell is updated before reading the event
		 * index from memory.  The controller needs to provide similar
		 * ordering to ensure the envent index is updated before reading
		 * the doorbell.
		 */
		mb();

		event_idx = le32_to_cpu(*dbbuf_ei);
		if (!nvme_dbbuf_need_event(event_idx, value, old_value))
			return false;
	}

	return true;
}

/*
 * Will slightly overestimate the number of pages needed.  This is OK
 * as it only leads to a small amount of wasted memory for the lifetime of
 * the I/O.
 */
static int nvme_pci_npages_prp(void)
{
	unsigned max_bytes = (NVME_MAX_KB_SZ * 1024) + NVME_CTRL_PAGE_SIZE;
	unsigned nprps = DIV_ROUND_UP(max_bytes, NVME_CTRL_PAGE_SIZE);
	return DIV_ROUND_UP(8 * nprps, NVME_CTRL_PAGE_SIZE - 8);
}

static int nvme_admin_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				unsigned int hctx_idx)
{
	struct nvme_dev *dev = to_nvme_dev(data);
	struct nvme_queue *nvmeq = &dev->queues[0];

	WARN_ON(hctx_idx != 0);
	WARN_ON(dev->admin_tagset.tags[0] != hctx->tags);

	hctx->driver_data = nvmeq;
	return 0;
}

static int nvme_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
			  unsigned int hctx_idx)
{
	struct nvme_dev *dev = to_nvme_dev(data);
	struct nvme_queue *nvmeq = &dev->queues[hctx_idx + 1];

	WARN_ON(dev->tagset.tags[hctx_idx] != hctx->tags);
	hctx->driver_data = nvmeq;
	return 0;
}

static int nvme_pci_init_request(struct blk_mq_tag_set *set,
		struct request *req, unsigned int hctx_idx,
		unsigned int numa_node)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);

	nvme_req(req)->ctrl = set->driver_data;
	nvme_req(req)->cmd = &iod->cmd;
	return 0;
}

static int queue_irq_offset(struct nvme_dev *dev)
{
	/* if we have more than 1 vec, admin queue offsets us by 1 */
	if (dev->num_vecs > 1)
		return 1;

	return 0;
}

static void nvme_pci_map_queues(struct blk_mq_tag_set *set)
{
	struct nvme_dev *dev = to_nvme_dev(set->driver_data);
	int i, qoff, offset;

	offset = queue_irq_offset(dev);
	for (i = 0, qoff = 0; i < set->nr_maps; i++) {
		struct blk_mq_queue_map *map = &set->map[i];

		map->nr_queues = dev->io_queues[i];
		if (!map->nr_queues) {
			BUG_ON(i == HCTX_TYPE_DEFAULT);
			continue;
		}

		/*
		 * The poll queue(s) doesn't have an IRQ (and hence IRQ
		 * affinity), so use the regular blk-mq cpu mapping
		 */
		map->queue_offset = qoff;
		if (i != HCTX_TYPE_POLL && offset)
        {   // KW_DB_20240612
            printk("%s(): Calling blk_mq_pci_map_queues() cpu[%d] offset[%d] (Ln %d)\n",
                __func__, current->on_cpu,  offset, __LINE__);  // KW_DB_20240612
			blk_mq_pci_map_queues(map, to_pci_dev(dev->dev), offset);
        }   // KW_DB_20240612
		else
        {   // KW_DB_20240612
            printk("%s(): Calling blk_mq_map_queues() offset[%d] (Ln %d)\n", __func__, offset, __LINE__);  // KW_DB_202406120
			blk_mq_map_queues(map);
        }   // KW_DB_20240612
		qoff += map->nr_queues;
		offset += map->nr_queues;
	}
}

/*
 * Write sq tail if we are asked to, or if the next command would wrap.
 */
static inline void nvme_write_sq_db(struct nvme_queue *nvmeq, bool write_sq)
{
	if (!write_sq) {
		u16 next_tail = nvmeq->sq_tail + 1;

		if (next_tail == nvmeq->q_depth)
			next_tail = 0;
		if (next_tail != nvmeq->last_sq_tail)
        {
            pr_err("%s(): Error! next_tail[%d] last_sq_tail[%d]. (Ln %d)\n",
                __func__, next_tail, nvmeq->last_sq_tail,__LINE__);    // KW_DB
			return;
        }
	}


#if 0   // KW_DB - Enable for xperiment only
    pr_info("%s(): Skipped Wr sq_tail[%d]. (Ln %d)\n", __func__, nvmeq->sq_tail, __LINE__);    // KW_DB
#else
	if (nvme_dbbuf_update_and_check_event(nvmeq->sq_tail,
			nvmeq->dbbuf_sq_db, nvmeq->dbbuf_sq_ei))
    {
        // KW_DB_20250408 - Enable the following for debugging.
        // pr_info("%s(): Wr sq[%d]_tail[%d] cpu[%d]. (Ln %d)\n",
            // __func__, nvmeq->qid, nvmeq->sq_tail, current->on_cpu, __LINE__);
		writel(nvmeq->sq_tail, nvmeq->q_db);
    }
	nvmeq->last_sq_tail = nvmeq->sq_tail;
#endif
}

static inline void nvme_sq_copy_cmd(struct nvme_queue *nvmeq,
				    struct nvme_command *cmd)
{
	memcpy(nvmeq->sq_cmds + (nvmeq->sq_tail << nvmeq->sqes),
		absolute_pointer(cmd), sizeof(*cmd));
	if (++nvmeq->sq_tail == nvmeq->q_depth)
		nvmeq->sq_tail = 0;
}

static void nvme_commit_rqs(struct blk_mq_hw_ctx *hctx)
{
	struct nvme_queue *nvmeq = hctx->driver_data;

	spin_lock(&nvmeq->sq_lock);
	if (nvmeq->sq_tail != nvmeq->last_sq_tail)
		nvme_write_sq_db(nvmeq, true);
	spin_unlock(&nvmeq->sq_lock);
}

static inline bool nvme_pci_use_sgls(struct nvme_dev *dev, struct request *req,
				     int nseg)
{
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	unsigned int avg_seg_size;

	avg_seg_size = DIV_ROUND_UP(blk_rq_payload_bytes(req), nseg);

	if (!nvme_ctrl_sgl_supported(&dev->ctrl))
		return false;
	if (!nvmeq->qid)
		return false;
	if (!sgl_threshold || avg_seg_size < sgl_threshold)
		return false;
	return true;
}

static void nvme_free_prps(struct nvme_dev *dev, struct request *req)
{
	const int last_prp = NVME_CTRL_PAGE_SIZE / sizeof(__le64) - 1;
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	dma_addr_t dma_addr = iod->first_dma;
	int i;

	for (i = 0; i < iod->nr_allocations; i++) {
		__le64 *prp_list = iod->list[i].prp_list;
		dma_addr_t next_dma_addr = le64_to_cpu(prp_list[last_prp]);

		dma_pool_free(dev->prp_page_pool, prp_list, dma_addr);
		dma_addr = next_dma_addr;
	}
}

static void nvme_unmap_data(struct nvme_dev *dev, struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
    
    ///pr_info("%s(): req(%p) (Ln %d)\n", __func__, req, __LINE__);    // KW_DB_20250220

	if (iod->dma_len) {
        ///pr_info("%s(): first_dma content[0x%llX]  (Ln %d)\n",
        ///    __func__, *(u64 *) iod->first_dma, __LINE__);    // KW_DB
		dma_unmap_page(dev->dev, iod->first_dma, iod->dma_len,
			       rq_dma_dir(req));
		return;
	}

	WARN_ON_ONCE(!iod->sgt.nents);

	dma_unmap_sgtable(dev->dev, &iod->sgt, rq_dma_dir(req), 0);

	if (iod->nr_allocations == 0)
		dma_pool_free(dev->prp_small_pool, iod->list[0].sg_list,
			      iod->first_dma);
	else if (iod->nr_allocations == 1)
		dma_pool_free(dev->prp_page_pool, iod->list[0].sg_list,
			      iod->first_dma);
	else
		nvme_free_prps(dev, req);
	mempool_free(iod->sgt.sgl, dev->iod_mempool);
}

static void nvme_print_sgl(struct scatterlist *sgl, int nents)
{
	int i;
	struct scatterlist *sg;

	for_each_sg(sgl, sg, nents, i) {
		dma_addr_t phys = sg_phys(sg);
		pr_warn("sg[%d] phys_addr:%pad offset:%d length:%d "
			"dma_address:%pad dma_length:%d\n",
			i, &phys, sg->offset, sg->length, &sg_dma_address(sg),
			sg_dma_len(sg));
	}
}

static blk_status_t nvme_pci_setup_prps(struct nvme_dev *dev,
		// struct request *req, struct nvme_rw_command *cmnd)       // KW_DB_20240627
        struct request *req, struct nvme_rw_command *cmnd, u16 qid) // KW_DB_20240627
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct dma_pool *pool;
	int length = blk_rq_payload_bytes(req);
	struct scatterlist *sg = iod->sgt.sgl;
	int dma_len = sg_dma_len(sg);
	u64 dma_addr = sg_dma_address(sg);
	int offset = dma_addr & (NVME_CTRL_PAGE_SIZE - 1);
	__le64 *prp_list;
	dma_addr_t prp_dma;
	int nprps, i;
    // KW_DB_20240627->
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    dma_addr_t  prp1_addr, prp2_addr;
    u32 realign_addr_lo;                            // KW_DB
    u32 realign_offset;                                     // KW_DB
    int widx;
    u32 addr32;
    // KW_DB_20240627<-
    u32 * ptr;                                     // KW_DB_20250220

	length -= (NVME_CTRL_PAGE_SIZE - offset);
	if (length <= 0) {
		iod->first_dma = 0;
		goto done;
	}

	dma_len -= (NVME_CTRL_PAGE_SIZE - offset);
	if (dma_len) {
		dma_addr += (NVME_CTRL_PAGE_SIZE - offset);
	} else {
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}

	if (length <= NVME_CTRL_PAGE_SIZE) {
		iod->first_dma = dma_addr;
		goto done;
	}

	nprps = DIV_ROUND_UP(length, NVME_CTRL_PAGE_SIZE);
    pr_info("%s(): nprps[%d] (Ln %d)\n", __func__, nprps, __LINE__);    // KW_DB
	if (nprps <= (256 / 8)) {
		pool = dev->prp_small_pool;
		iod->nr_allocations = 0;
	} else {
		pool = dev->prp_page_pool;
		iod->nr_allocations = 1;
	}

	prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
	if (!prp_list) {
		iod->nr_allocations = -1;
		return BLK_STS_RESOURCE;
	}
	iod->list[0].prp_list = prp_list;
	iod->first_dma = prp_dma;
	i = 0;
	for (;;) {
		if (i == NVME_CTRL_PAGE_SIZE >> 3) {
			__le64 *old_prp_list = prp_list;
			prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
			if (!prp_list)
				goto free_prps;
			iod->list[iod->nr_allocations++].prp_list = prp_list;
			prp_list[0] = old_prp_list[i - 1];
#if (USE_MW_PRP)    // KW_DB_20250224
			old_prp_list[i - 1] = cpu_to_le64(dev->mw0_addr + prp_dma);
            dev_info(dev->ctrl.device, "%s() New dma_addr[0x%llX] slot[%d] (Ln %d)\n",
                __func__, (u64) old_prp_list[i - 1], i-1, __LINE__); // KW_DB            
#else
			old_prp_list[i - 1] = cpu_to_le64(prp_dma);
#endif  
			i = 1;
		}
#if (USE_MW_PRP)    // KW_DB_20250224
		prp_list[i++] = cpu_to_le64(dev->mw0_addr + dma_addr);
        // KW_DB_20250408 - Enable the following for debugging.
        // dev_info(dev->ctrl.device, "%s() New dma_addr[0x%llX] slot[%d] (Ln %d)\n",
            // __func__, (u64) cpu_to_le64(dev->mw0_addr + dma_addr), i, __LINE__);
#else
		prp_list[i++] = cpu_to_le64(dma_addr);
#endif
		dma_len -= NVME_CTRL_PAGE_SIZE;
		dma_addr += NVME_CTRL_PAGE_SIZE;
		length -= NVME_CTRL_PAGE_SIZE;
		if (length <= 0)
			break;
		if (dma_len > 0)
			continue;
		if (unlikely(dma_len < 0))
			goto bad_sgl;
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}
done:
    ///pr_info("%s(): Here. dptr... (Ln %d)\n", __func__, __LINE__);    // KW_DB




	cmnd->dptr.prp1 = cpu_to_le64(sg_dma_address(iod->sgt.sgl));
	cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma);

// KW_DB_20241211->
#if (USE_MW_PRP)
    cmnd->dptr.prp1 = cpu_to_le64(dev->mw0_addr + sg_dma_address(iod->sgt.sgl));
    cmnd->dptr.prp2 = cpu_to_le64(dev->mw0_addr + iod->first_dma);
    
    #if 0   // KW_DB_20250328 - Enable for debugging.
    // KW_DB_20250220->
    ptr = (u32*) cmnd;
    for (i=0; i<16; i++)
    {
        pr_info("%s(): cmd dword%02d[0x%08X]. (Ln %d)\n",__func__,  i, *ptr, __LINE__); // KW_DB
        ptr++;
    }
    // KW_DB_20250220<-
    #endif  // KW_DB_20250328
    
#else   // USE_MW_PRP
// KW_DB_20241211<-

// KW_DB_20240627->
#if 1 // Disable for debugging
    dev_err(dev->ctrl.device, "%s() sg_dma_address[0x%llX] (Ln %d)\n",
        __func__, sg_dma_address(iod->sgt.sgl) , __LINE__); // KW_DB
    dev_err(dev->ctrl.device, "%s() first_dma[0x%llX] (Ln %d)\n",
        __func__, iod->first_dma , __LINE__); // KW_DB
    dev_err(dev->ctrl.device, "%s() prp1[0x%llX] (Ln %d)\n",
        __func__, cmnd->dptr.prp1 , __LINE__); // KW_DB
    dev_err(dev->ctrl.device, "%s() prp2[0x%llX] (Ln %d)\n",
        __func__, cmnd->dptr.prp2 , __LINE__); // KW_DB

    // Request reverse mapping with LUT FOR prp1
    realign_addr_lo = my_round_down(dev, (u32)(cmnd->dptr.prp1 & 0xFFFFFFFF));
    realign_offset = ((u32)(cmnd->dptr.prp1 & 0xFFFFFFFF)) - realign_addr_lo;

    getFreeWidx(dev, &widx);    // DYNAMIC_WIDX
    dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated for prp1. (Ln %d)\n",
        __func__, widx, __LINE__); // KW_DB
    addr32 = realign_addr_lo | qid;
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_LO, addr32);

    addr32 = (u32)(cmnd->dptr.prp1 >> 32);
    addr32 |= ((u32)qid << ADDR_HI_QID_SHIFT);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_HI, addr32);

    prp1_addr = dev->dnb_addr + ((widx+1) * SZ_32K) + realign_offset;
    cmnd->dptr.prp1 = prp1_addr;
    dev_err(dev->ctrl.device, "%s() Wrote addr[0x%llX] to prp1 (Ln %d)\n",
        __func__, prp1_addr, __LINE__); // KW_DB


    // Request reverse mapping with LUT FOR prp2
    realign_addr_lo = my_round_down(dev, (u32)(cmnd->dptr.prp2 & 0xFFFFFFFF));
    realign_offset = ((u32)(cmnd->dptr.prp2 & 0xFFFFFFFF)) - realign_addr_lo;

    getFreeWidx(dev, &widx);    // DYNAMIC_WIDX
    dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated for prp2. (Ln %d)\n",
        __func__, widx, __LINE__); // KW_DB
    addr32 = realign_addr_lo | qid;
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_LO, addr32);

    addr32 = (u32)(cmnd->dptr.prp2 >> 32);
    addr32 |= ((u32)qid << ADDR_HI_QID_SHIFT);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_HI, addr32);

    prp2_addr = dev->dnb_addr + ((widx+1) * SZ_32K) + realign_offset;
    cmnd->dptr.prp2 = prp2_addr;
    dev_err(dev->ctrl.device, "%s() Wrote addr[0x%llX] to prp2 (Ln %d)\n",
        __func__, prp2_addr, __LINE__); // KW_DB
#endif
// KW_DB_20240627<-

#endif  // USE_MW_PRP   // KW_DB_20241211

	return BLK_STS_OK;
free_prps:
	nvme_free_prps(dev, req);
	return BLK_STS_RESOURCE;
bad_sgl:
	WARN(DO_ONCE(nvme_print_sgl, iod->sgt.sgl, iod->sgt.nents),
			"Invalid SGL for payload:%d nents:%d\n",
			blk_rq_payload_bytes(req), iod->sgt.nents);
	return BLK_STS_IOERR;
}

#if (USE_MW_PRP)    // KW_DB_20250224
static void nvme_pci_sgl_set_data(struct nvme_dev *dev,
                                  struct nvme_sgl_desc *sge,
		                          struct scatterlist *sg)
{
	sge->addr = cpu_to_le64(dev->mw0_addr + sg_dma_address(sg));
	sge->length = cpu_to_le32(sg_dma_len(sg));
	sge->type = NVME_SGL_FMT_DATA_DESC << 4;
}

static void nvme_pci_sgl_set_seg(struct nvme_dev *dev,
                                 struct nvme_sgl_desc *sge,
		                         dma_addr_t dma_addr,
                                 int entries)
{
	sge->addr = cpu_to_le64(dev->mw0_addr + dma_addr);
	sge->length = cpu_to_le32(entries * sizeof(*sge));
	sge->type = NVME_SGL_FMT_LAST_SEG_DESC << 4;
}
#else
static void nvme_pci_sgl_set_data(struct nvme_sgl_desc *sge,
		struct scatterlist *sg)
{
	sge->addr = cpu_to_le64(sg_dma_address(sg));
	sge->length = cpu_to_le32(sg_dma_len(sg));
	sge->type = NVME_SGL_FMT_DATA_DESC << 4;
}

static void nvme_pci_sgl_set_seg(struct nvme_sgl_desc *sge,
		dma_addr_t dma_addr, int entries)
{
	sge->addr = cpu_to_le64(dma_addr);
	sge->length = cpu_to_le32(entries * sizeof(*sge));
	sge->type = NVME_SGL_FMT_LAST_SEG_DESC << 4;
}
#endif

static blk_status_t nvme_pci_setup_sgls(struct nvme_dev *dev,
		struct request *req, struct nvme_rw_command *cmd)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct dma_pool *pool;
	struct nvme_sgl_desc *sg_list;
	struct scatterlist *sg = iod->sgt.sgl;
	unsigned int entries = iod->sgt.nents;
	dma_addr_t sgl_dma;
	int i = 0;

	/* setting the transfer type as SGL */
	cmd->flags = NVME_CMD_SGL_METABUF;

	if (entries == 1) {
#if (USE_MW_PRP)    // KW_DB_20250224
		nvme_pci_sgl_set_data(dev, &cmd->dptr.sgl, sg);
#else
		nvme_pci_sgl_set_data(&cmd->dptr.sgl, sg);
#endif
		return BLK_STS_OK;
	}

	if (entries <= (256 / sizeof(struct nvme_sgl_desc))) {
		pool = dev->prp_small_pool;
		iod->nr_allocations = 0;
	} else {
		pool = dev->prp_page_pool;
		iod->nr_allocations = 1;
	}

	sg_list = dma_pool_alloc(pool, GFP_ATOMIC, &sgl_dma);
	if (!sg_list) {
		iod->nr_allocations = -1;
		return BLK_STS_RESOURCE;
	}

	iod->list[0].sg_list = sg_list;
	iod->first_dma = sgl_dma;

#if (USE_MW_PRP)    // KW_DB_20250224
	nvme_pci_sgl_set_seg(dev, &cmd->dptr.sgl, sgl_dma, entries);
#else
	nvme_pci_sgl_set_seg(&cmd->dptr.sgl, sgl_dma, entries);
#endif
	do {
#if (USE_MW_PRP)    // KW_DB_20250224
		nvme_pci_sgl_set_data(dev, &sg_list[i++], sg);
#else        
		nvme_pci_sgl_set_data(&sg_list[i++], sg);
#endif
		sg = sg_next(sg);
	} while (--entries > 0);

	return BLK_STS_OK;
}

// KW_DB->
/**
 * Update SQ and CQ size based on NTB LUT size alignment requirement.
 */
static void update_queues_size(struct nvme_dev *dev, struct nvme_queue *nvmeq)
{
    nvmeq->sq_size_orig = nvmeq->q_depth * nvmeq->sqes;
    nvmeq->cq_size_orig = nvmeq->q_depth * sizeof(struct nvme_completion);
    nvmeq->sq_size_new = round_up(nvmeq->sq_size_orig, dev->lut_size_align);
    nvmeq->cq_size_new = round_up(nvmeq->cq_size_orig, dev->lut_size_align);

    // dev_info(dev->ctrl.device, "%s() sq_size [0x%lX]=>[0x%lX] (Ln %d)\n", __func__, nvmeq->sq_size_orig, nvmeq->sq_size_new, __LINE__); // KW_DB
    // dev_info(dev->ctrl.device, "%s() cq_size [0x%lX]=>[0x%lX] (Ln %d)\n", __func__, nvmeq->cq_size_orig, nvmeq->cq_size_new, __LINE__); // KW_DB
    dev_info(dev->ctrl.device, "%s() sq_size[0x%lX]=>[0x%lX] cq_size[0x%lX]=>[0x%lX] (Ln %d)\n",
        __func__, nvmeq->sq_size_orig, nvmeq->sq_size_new,
        nvmeq->cq_size_orig, nvmeq->cq_size_new, __LINE__); // KW_DB
}

/**
 * DMA address alignment check.
 */
static int check_addr_align(struct nvme_dev *dev, dma_addr_t dma_addr)
{
	if (!IS_ALIGNED(dma_addr, dev->lut_xlat_align)) {
		dev_err(dev->ctrl.device, "DMA address[0x%llX] not [0x%X] aligned!\n",
            dma_addr, dev->lut_xlat_align);
		return -1;
	}
    return 0;
}

/**
 *
 */
static u32 my_round_down(struct nvme_dev *dev, u32 addr)
{
    u32 new_val;

    new_val = round_down(addr, dev->lut_xlat_align);

    dev_info(dev->ctrl.device, "%s() orig_val[0x%X] new_val[0x%X] (Ln %d)\n",
        __func__, addr, new_val, __LINE__); // KW_DB

    return new_val;
}

/**
 *
 */
static void update_align_info(struct nvme_dev *dev, int widx)
{
    struct pci_dev *pdev = to_pci_dev(dev->dev);

    /* Get the LUT alignment info. */
    pci_write_config_dword(pdev, NTB_NVME_ALIGN_SET_WIDX, (u32) widx);
    pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_XLAT, &dev->lut_xlat_align);
    pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE, &dev->lut_size_align);
    pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE_MAX, &dev->lut_size_max);
    dev_info(dev->ctrl.device, "%s() lut_xlat_align[0x%X] lut_size_align[0x%X] lut_size_max[0x%X] (Ln %d)\n",
        __func__, dev->lut_xlat_align, dev->lut_size_align, dev->lut_size_max, __LINE__); // KW_DB

}
// KW_DB<-
#if (DYNAMIC_WIDX)
static blk_status_t nvme_setup_prp_simple(struct nvme_dev *dev,
		struct request *req, struct nvme_rw_command *cmnd,
		struct bio_vec *bv, u16 qid)
#else
static blk_status_t nvme_setup_prp_simple(struct nvme_dev *dev,
		struct request *req, struct nvme_rw_command *cmnd,
		struct bio_vec *bv)
#endif
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	unsigned int offset = bv->bv_offset & (NVME_CTRL_PAGE_SIZE - 1);
	unsigned int first_prp_len = NVME_CTRL_PAGE_SIZE - offset;
#if 1 // Disable for debugging
    struct pci_dev *pdev = to_pci_dev(dev->dev);    // KW_DB
    dma_addr_t  prp1_addr;                          // KW_DB
    u32 realign_addr_lo;                            // KW_DB
    u32 realign_offset;                                     // KW_DB
    int widx;                                       // KW_DB
    int i;                                     // KW_DB
    u32 * ptr;                                     // KW_DB
#if (USE_DBUF)
    struct nvme_queue *nvmeq;
#endif
#if (DYNAMIC_WIDX)
    u32 addr32;
#endif
#endif

    /// KW_DB_20241211->
	iod->first_dma = dma_map_bvec(dev->dev, bv, rq_dma_dir(req), 0);
    ///iod->first_dma = dma_map_bvec(dev->dev, bv, rq_dma_dir(req), DMA_ATTR_FORCE_CONTIGUOUS);
    /// KW_DB_20241211<-
    
	if (dma_mapping_error(dev->dev, iod->first_dma))
		return BLK_STS_RESOURCE;
	iod->dma_len = bv->bv_len;

// KW_DB->
#if 1 // Disable for debugging

#if (USE_DBUF)
    nvmeq = &dev->queues[0];

    pr_err("%s(): dBufDmaAddr[0x%llX] (Ln %d)\n",
        __func__, nvmeq->dBufDmaAddr, __LINE__);    // KW_DB
    if (check_addr_align(dev, nvmeq->dBufDmaAddr))
        dev_err(dev->ctrl.device, "PRP aligment failed!\n");
    // Request reverse mapping with LUT.
    realign_addr_lo = my_round_down(dev, (u32)(nvmeq->dBufDmaAddr & 0xFFFFFFFF));
    realign_offset = ((u32)(nvmeq->dBufDmaAddr & 0xFFFFFFFF)) - realign_addr_lo;
    {
        u32 *ptr = nvmeq->dBuf;
        memset(ptr, 0xA5, 1024);
        *ptr = 0x12345678;
    }
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_LO, realign_addr_lo);
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_HI, (u32)(nvmeq->dBufDmaAddr >> 32));
    // Use widx 9 for prp1, Update with remote addresses.
    #if 0// For Wei's test
    widx = 7;
    prp1_addr = dev->dnb_addr + ((widx+1) * SZ_32K);
    #else
    widx = 9;
    prp1_addr = dev->dnb_addr + ((widx+1) * SZ_32K) + realign_offset;
    #endif
    cmnd->dptr.prp1 = cpu_to_le64(prp1_addr);
#else

// KW_DB_20241211->
#if (USE_MW_PRP)
    // KW_DB_20250408 - Enable the following for debugging.
    // pr_info("%s(): first_dma[0x%llX] first_prp_len[%d]. dptr... (Ln %d)\n",
        // __func__, iod->first_dma, first_prp_len, __LINE__);
    prp1_addr = dev->mw0_addr + iod->first_dma;
#else   // USE_MW_PRP
// KW_DB_20241211<-
    pr_err("%s(): first_dma[0x%llX] first_prp_len[%d]. dptr... (Ln %d)\n",
        __func__, iod->first_dma, first_prp_len, __LINE__);    // KW_DB

    if (check_addr_align(dev, iod->first_dma))
        dev_err(dev->ctrl.device, "PRP aligment failed!\n");

    // Request reverse mapping with LUT.
    realign_addr_lo = my_round_down(dev, (u32)(iod->first_dma & 0xFFFFFFFF));
    realign_offset = ((u32)(iod->first_dma & 0xFFFFFFFF)) - realign_addr_lo;


#if (DYNAMIC_WIDX)

    getFreeWidx(dev, &widx);
    dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n",
        __func__, widx, __LINE__); // KW_DB
    ///addr32 = realign_addr_lo | (widx << ADDR_LO_WIDX_SHIFT);
    addr32 = realign_addr_lo | qid;
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_LO, addr32);

    addr32 = (u32)(iod->first_dma >> 32);
    addr32 |= ((u32)qid << ADDR_HI_QID_SHIFT);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_HI, addr32);
#else
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_LO, realign_addr_lo);
    pci_write_config_dword(pdev, NTB_NVME_PRP1_ADDR_HI, (u32)(iod->first_dma >> 32));

    // Use widx 9 for prp1, Update with remote addresses.
    widx = 9;
#endif

    prp1_addr = dev->dnb_addr + ((widx+1) * SZ_32K) + realign_offset;
    // pr_info("%s(): Virtual prp1_addr[0x%llX]. dptr... (Ln %d)\n",
        // __func__, prp1_addr, __LINE__);    // KW_DB
#endif  // USE_MW_PRP   // KW_DB_20241211
    cmnd->dptr.prp1 = cpu_to_le64(prp1_addr);


#endif

    // KW_DB_20250408 - Enable the following for debugging.
    // dev_info(dev->ctrl.device, "%s() Wrote addr[0x%llX] to prp1 (Ln %d)\n",
        // __func__, prp1_addr, __LINE__);

	if (bv->bv_len > first_prp_len)
    {
        pr_warn("%s(): bv_len[%d]. dptr... (Ln %d)\n", __func__, bv->bv_len, __LINE__);    // KW_DB
		cmnd->dptr.prp2 = cpu_to_le64(prp1_addr + first_prp_len);
    }
	else
		cmnd->dptr.prp2 = 0;


    #if 0   // KW_DB_20250328 - Enable for debugging.
    ptr = (u32*) cmnd;
    for (i=0; i<16; i++)
    {
        pr_info("%s(): cmd dword%02d[0x%08X]. (Ln %d)\n",__func__,  i, *ptr, __LINE__); // KW_DB
        ptr++;
    }
    #endif  // KW_DB_20250328

#else
// KW_DB<-

	cmnd->dptr.prp1 = cpu_to_le64(iod->first_dma);
	if (bv->bv_len > first_prp_len)
		cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma + first_prp_len);
	else
		cmnd->dptr.prp2 = 0;
#endif  // KW_DB
	return BLK_STS_OK;
}

static blk_status_t nvme_setup_sgl_simple(struct nvme_dev *dev,
		struct request *req, struct nvme_rw_command *cmnd,
		struct bio_vec *bv)
{
    u32 * ptr;                                      // KW_DB_20250220
    int i;                                          // KW_DB_20250220
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);

    ///pr_info("%s(): Calling dma_map_bvec() (Ln %d)\n", __func__, __LINE__);    // KW_DB
	iod->first_dma = dma_map_bvec(dev->dev, bv, rq_dma_dir(req), 0);
	if (dma_mapping_error(dev->dev, iod->first_dma))
		return BLK_STS_RESOURCE;
	iod->dma_len = bv->bv_len;

    pr_info("%s(): Here. dptr... (Ln %d)\n", __func__, __LINE__);    // KW_DB
	cmnd->flags = NVME_CMD_SGL_METABUF;
#if (USE_MW_PRP)    // KW_DB_20250220
    cmnd->dptr.sgl.addr = cpu_to_le64(dev->mw0_addr + iod->first_dma);
#else
	cmnd->dptr.sgl.addr = cpu_to_le64(iod->first_dma);
#endif
	cmnd->dptr.sgl.length = cpu_to_le32(iod->dma_len);
	cmnd->dptr.sgl.type = NVME_SGL_FMT_DATA_DESC << 4;
    
    // KW_DB_20250220->
    ptr = (u32*) cmnd;
    for (i=0; i<16; i++)
    {
        pr_info("%s(): cmd dword%02d[0x%08X]. (Ln %d)\n",__func__,  i, *ptr, __LINE__); // KW_DB
        ptr++;
    }
    // KW_DB_20250220<-
	return BLK_STS_OK;
}

static blk_status_t nvme_map_data(struct nvme_dev *dev, struct request *req,
		struct nvme_command *cmnd)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret = BLK_STS_RESOURCE;
	int rc;

	if (blk_rq_nr_phys_segments(req) == 1) {
		struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
		struct bio_vec bv = req_bvec(req);

		if (!is_pci_p2pdma_page(bv.bv_page)) {
			if (bv.bv_offset + bv.bv_len <= NVME_CTRL_PAGE_SIZE * 2)
#if (DYNAMIC_WIDX)
				return nvme_setup_prp_simple(dev, req,
							     &cmnd->rw, &bv, nvmeq->qid);
#else
				return nvme_setup_prp_simple(dev, req,
							     &cmnd->rw, &bv);
#endif

			if (nvmeq->qid && sgl_threshold &&
			    nvme_ctrl_sgl_supported(&dev->ctrl))
				return nvme_setup_sgl_simple(dev, req,
							     &cmnd->rw, &bv);
		}
	}

	iod->dma_len = 0;
	iod->sgt.sgl = mempool_alloc(dev->iod_mempool, GFP_ATOMIC);
	if (!iod->sgt.sgl)
		return BLK_STS_RESOURCE;
	sg_init_table(iod->sgt.sgl, blk_rq_nr_phys_segments(req));
	iod->sgt.orig_nents = blk_rq_map_sg(req->q, req, iod->sgt.sgl);
	if (!iod->sgt.orig_nents)
		goto out_free_sg;

	rc = dma_map_sgtable(dev->dev, &iod->sgt, rq_dma_dir(req),
			     DMA_ATTR_NO_WARN);
	if (rc) {
		if (rc == -EREMOTEIO)
			ret = BLK_STS_TARGET;
		goto out_free_sg;
	}

	if (nvme_pci_use_sgls(dev, req, iod->sgt.nents))
		ret = nvme_pci_setup_sgls(dev, req, &cmnd->rw);
	else
        // KW_DB_20240627->
		// ret = nvme_pci_setup_prps(dev, req, &cmnd->rw);
        {
            struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
            ret = nvme_pci_setup_prps(dev, req, &cmnd->rw, nvmeq->qid);
        }
        // KW_DB_20240627<-
	if (ret != BLK_STS_OK)
		goto out_unmap_sg;
	return BLK_STS_OK;

out_unmap_sg:
	dma_unmap_sgtable(dev->dev, &iod->sgt, rq_dma_dir(req), 0);
out_free_sg:
	mempool_free(iod->sgt.sgl, dev->iod_mempool);
	return ret;
}

static blk_status_t nvme_map_metadata(struct nvme_dev *dev, struct request *req,
		struct nvme_command *cmnd)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);

    pr_info("%s(): Calling dma_map_bvec() (Ln %d)\n", __func__, __LINE__);    // KW_DB
	iod->meta_dma = dma_map_bvec(dev->dev, rq_integrity_vec(req),
			rq_dma_dir(req), 0);
	if (dma_mapping_error(dev->dev, iod->meta_dma))
		return BLK_STS_IOERR;
#if (USE_MW_PRP)    // KW_DB_20250224
	cmnd->rw.metadata = cpu_to_le64(dev->mw0_addr + iod->meta_dma);
#else
	cmnd->rw.metadata = cpu_to_le64(iod->meta_dma);
#endif
	return BLK_STS_OK;
}

static blk_status_t nvme_prep_rq(struct nvme_dev *dev, struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret;

	iod->aborted = false;
	iod->nr_allocations = -1;
	iod->sgt.nents = 0;

	ret = nvme_setup_cmd(req->q->queuedata, req);
	if (ret)
		return ret;

	if (blk_rq_nr_phys_segments(req)) {
		ret = nvme_map_data(dev, req, &iod->cmd);
		if (ret)
			goto out_free_cmd;
	}

	if (blk_integrity_rq(req)) {
		ret = nvme_map_metadata(dev, req, &iod->cmd);
		if (ret)
			goto out_unmap_data;
	}

	nvme_start_request(req);
	return BLK_STS_OK;
out_unmap_data:
	nvme_unmap_data(dev, req);
out_free_cmd:
	nvme_cleanup_cmd(req);
	return ret;
}

// KW_DB->
static void read_msix_cap(struct nvme_dev *dev, const char* func, const int line)
{
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    u32             dataB0, dateB4;

    pci_read_config_dword(pdev, 0xB0, &dataB0);
    pci_read_config_dword(pdev, 0xB4, &dateB4);
    pr_info("%s(): PciCfg 0xB0[0x%X] 0xB4[0x%X] (Ln %d)\n", func, dataB0, dateB4, line);
}

static void set_msix_enable(struct nvme_dev *dev, const char* func, const int line)
{
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    u32             dataB0, date_new;

    pci_read_config_dword(pdev, 0xB0, &dataB0);
    pr_info("%s(): PciCfg 0xB0[0x%X] original (Ln %d)\n", func, dataB0, line);
    
    date_new = dataB0 | 0x80000000;
    pci_write_config_dword(pdev, 0xB0, date_new);
    
    pci_read_config_dword(pdev, 0xB0, &dataB0);
    pr_info("%s(): PciCfg 0xB0[0x%X] new (Ln %d)\n", func, dataB0, line);    
}

// KW_DB<-

/*
 * NOTE: ns is NULL when called on the admin queue.
 */
static blk_status_t nvme_queue_rq(struct blk_mq_hw_ctx *hctx,
			 const struct blk_mq_queue_data *bd)
{
	struct nvme_queue *nvmeq = hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;
	struct request *req = bd->rq;
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret;

    // KW_DB_20250408 - Enable the following for debugging.
    // pr_info("%s(): req(%p) (Ln %d)\n", __func__, req, __LINE__);
	/*
	 * We should not need to do this, but we're still using this to
	 * ensure we can drain requests on a dying queue.
	 */
	if (unlikely(!test_bit(NVMEQ_ENABLED, &nvmeq->flags)))
		return BLK_STS_IOERR;

	if (unlikely(!nvme_check_ready(&dev->ctrl, req, true)))
		return nvme_fail_nonready_command(&dev->ctrl, req);

	ret = nvme_prep_rq(dev, req);
	if (unlikely(ret))
		return ret;
    
    // KW_DB->
    #if 0   // Enable for debugging.
    {
        struct pci_dev *pdev = to_pci_dev(dev->dev);
        u32 dataB0, dateB4;
        
        pci_read_config_dword(pdev, 0xB0, &dataB0);
        pci_read_config_dword(pdev, 0xB4, &dateB4);
        pr_info("%s(): Dfg 0xB0[0x%08X] 0xB4[0x%08X] (Ln %d)\n", __func__, dataB0, dateB4, __LINE__);
    }
    #endif
    // KW_DB<-
    
	spin_lock(&nvmeq->sq_lock);
	nvme_sq_copy_cmd(nvmeq, &iod->cmd);
    // KW_DB_20250408 - Enable the following for debugging.
    // pr_info("%s(): Calling nvme_write_sq_db() opcode[0x%X]. (Ln %d)\n",
        // __func__, iod->cmd.common.opcode, __LINE__);
	nvme_write_sq_db(nvmeq, bd->last);
	spin_unlock(&nvmeq->sq_lock);
	return BLK_STS_OK;
}

static void nvme_submit_cmds(struct nvme_queue *nvmeq, struct request **rqlist)
{
    // KW_DB->
    struct nvme_dev *dev = nvmeq->dev;
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    #if 0   // Enable for debugging.
    u32 dataB0, dateB4;
    
    pci_read_config_dword(pdev, 0xB0, &dataB0);
    pci_read_config_dword(pdev, 0xB4, &dateB4);
    pr_info("%s(): Dfg 0xB0[0x%X] 0xB4[0x%X] (Ln %d)\n", __func__, dataB0, dateB4, __LINE__);
    #endif
    // KW_DB<-
	spin_lock(&nvmeq->sq_lock);
	while (!rq_list_empty(*rqlist)) {
		struct request *req = rq_list_pop(rqlist);
		struct nvme_iod *iod = blk_mq_rq_to_pdu(req);

		nvme_sq_copy_cmd(nvmeq, &iod->cmd);
        // KW_DB_20250408 - Enable the following for debugging.
        // pr_info("%s(): opcode[0x%X].  (Ln %d)\n",
            // __func__, iod->cmd.common.opcode, __LINE__);
	}
    ///pr_info("%s(): Calling nvme_write_sq_db(). (Ln %d)\n", __func__, __LINE__);    // KW_DB
	nvme_write_sq_db(nvmeq, true);
	spin_unlock(&nvmeq->sq_lock);
}

static bool nvme_prep_rq_batch(struct nvme_queue *nvmeq, struct request *req)
{
	/*
	 * We should not need to do this, but we're still using this to
	 * ensure we can drain requests on a dying queue.
	 */
	if (unlikely(!test_bit(NVMEQ_ENABLED, &nvmeq->flags)))
		return false;
	if (unlikely(!nvme_check_ready(&nvmeq->dev->ctrl, req, true)))
		return false;

	req->mq_hctx->tags->rqs[req->tag] = req;
	return nvme_prep_rq(nvmeq->dev, req) == BLK_STS_OK;
}

static void nvme_queue_rqs(struct request **rqlist)
{
	struct request *req, *next, *prev = NULL;
	struct request *requeue_list = NULL;

    ///pr_info("%s(): Entered. (Ln %d)\n", __func__, __LINE__);    // KW_DB
	rq_list_for_each_safe(rqlist, req, next) {
		struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
        
        pr_info("%s(): req(%p) (Ln %d)\n", __func__, req, __LINE__);    // KW_DB_20250220

		if (!nvme_prep_rq_batch(nvmeq, req)) {
			/* detach 'req' and add to remainder list */
			rq_list_move(rqlist, &requeue_list, req, prev);

			req = prev;
			if (!req)
				continue;
		}

		if (!next || req->mq_hctx != next->mq_hctx) {
			/* detach rest of list, and submit */
			req->rq_next = NULL;
			nvme_submit_cmds(nvmeq, rqlist);
			*rqlist = next;
			prev = NULL;
		} else
			prev = req;
	}

	*rqlist = requeue_list;
}

static __always_inline void nvme_pci_unmap_rq(struct request *req)
{
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;

	if (blk_integrity_rq(req)) {
	        struct nvme_iod *iod = blk_mq_rq_to_pdu(req);

		dma_unmap_page(dev->dev, iod->meta_dma,
			       rq_integrity_vec(req)->bv_len, rq_dma_dir(req));
	}

	if (blk_rq_nr_phys_segments(req))
		nvme_unmap_data(dev, req);
}

static void nvme_pci_complete_rq(struct request *req)
{
	nvme_pci_unmap_rq(req);
	nvme_complete_rq(req);
}

static void nvme_pci_complete_batch(struct io_comp_batch *iob)
{
    // KW_DB_20250408 - Enable the following for debugging.
    // pr_info("%s() Entered. (Ln %d)\n", __func__, __LINE__);
	nvme_complete_batch(iob, nvme_pci_unmap_rq);
}

/* We read the CQE phase first to check if the rest of the entry is valid */
static inline bool nvme_cqe_pending(struct nvme_queue *nvmeq)
{
	struct nvme_completion *hcqe = &nvmeq->cqes[nvmeq->cq_head];

	return (le16_to_cpu(READ_ONCE(hcqe->status)) & 1) == nvmeq->cq_phase;
}

static inline void nvme_ring_cq_doorbell(struct nvme_queue *nvmeq)
{
	u16 head = nvmeq->cq_head;

	if (nvme_dbbuf_update_and_check_event(head, nvmeq->dbbuf_cq_db,
					      nvmeq->dbbuf_cq_ei))
		writel(head, nvmeq->q_db + nvmeq->dev->db_stride);
}

static inline struct blk_mq_tags *nvme_queue_tagset(struct nvme_queue *nvmeq)
{
	if (!nvmeq->qid)
		return nvmeq->dev->admin_tagset.tags[0];
	return nvmeq->dev->tagset.tags[nvmeq->qid - 1];
}

static inline void nvme_handle_cqe(struct nvme_queue *nvmeq,
				   struct io_comp_batch *iob, u16 idx)
{
	struct nvme_completion *cqe = &nvmeq->cqes[idx];
	__u16 command_id = READ_ONCE(cqe->command_id);
    // KW_DB->
    __u16 status = READ_ONCE(cqe->status);
    __u16 sq_id = READ_ONCE(cqe->sq_id);
    // KW_DB<-
	struct request *req;

    // KW_DB_20250408 - Enable the following for debugging.
    // pr_info("%s() idx[%d] command_id[0x%X] (Ln %d)\n", __func__, idx, command_id, __LINE__);
    
    // KW_DB->
    if (status & 0xFE)
    {
        pr_err("%s() idx[%d] sq_id[%d] command_id[0x%X] status[0x%X] sc[0x%X] sct[0x%X] (Ln %d)\n",
            __func__, idx, sq_id,  command_id, status, status & 0xfe, status >> 9 & 7, __LINE__);  // KW_DB
    }
    // KW_DB<-
    
	/*
	 * AEN requests are special as they don't time out and can
	 * survive any kind of queue freeze and often don't respond to
	 * aborts.  We don't even bother to allocate a struct request
	 * for them but rather special case them here.
	 */
	if (unlikely(nvme_is_aen_req(nvmeq->qid, command_id))) {
		nvme_complete_async_event(&nvmeq->dev->ctrl,
				cqe->status, &cqe->result);
		return;
	}

	req = nvme_find_rq(nvme_queue_tagset(nvmeq), command_id);
	if (unlikely(!req)) {
		dev_warn(nvmeq->dev->ctrl.device,
			"invalid id %d completed on queue %d\n",
			command_id, le16_to_cpu(cqe->sq_id));
		return;
	}

	trace_nvme_sq(req, cqe->sq_head, nvmeq->sq_tail);
	if (!nvme_try_complete_req(req, cqe->status, cqe->result) &&
	    !blk_mq_add_to_batch(req, iob, nvme_req(req)->status,
					nvme_pci_complete_batch))
		nvme_pci_complete_rq(req);
}

static inline void nvme_update_cq_head(struct nvme_queue *nvmeq)
{
	u32 tmp = nvmeq->cq_head + 1;

	if (tmp == nvmeq->q_depth) {
		nvmeq->cq_head = 0;
		nvmeq->cq_phase ^= 1;
	} else {
		nvmeq->cq_head = tmp;
	}
}

static inline int nvme_poll_cq(struct nvme_queue *nvmeq,
			       struct io_comp_batch *iob)
{
	int found = 0;

	while (nvme_cqe_pending(nvmeq)) {
		found++;
        // KW_DB_20250408 - Enable the following for debugging.
        pr_info("%s() found[%d] qid[%d] (Ln %d)\n", __func__, found, nvmeq->qid, __LINE__);
		/*
		 * load-load control dependency between phase and the rest of
		 * the cqe requires a full read memory barrier
		 */
		dma_rmb();
		nvme_handle_cqe(nvmeq, iob, nvmeq->cq_head);
		nvme_update_cq_head(nvmeq);
	}

	if (found)
		nvme_ring_cq_doorbell(nvmeq);
    // KW_DB->
    else
    {
        pr_err("%s() Not found qid[%d] (Ln %d)\n", __func__, nvmeq->qid, __LINE__);
    }
    // KW_DB<-
	return found;
}

static irqreturn_t nvme_irq(int irq, void *data)
{
	struct nvme_queue *nvmeq = data;
	DEFINE_IO_COMP_BATCH(iob);

    pr_info("%s() cpu[%d] qid[%d] irq[0x%X] (Ln %d)\n",
        __func__, current->on_cpu, nvmeq->qid, irq, __LINE__);  // KW_DB
        
    ///dump_stack();   // KW_DB_20241029

	if (nvme_poll_cq(nvmeq, &iob)) {
		if (!rq_list_empty(iob.req_list))
			nvme_pci_complete_batch(&iob);
        pr_info("%s() IRQ_HANDLED \n", __func__);  // KW_DB_20240612
		return IRQ_HANDLED;
	}
    pr_err("%s() IRQ_NONE \n", __func__);  // KW_DB_20240612
	return IRQ_NONE;
}

static irqreturn_t nvme_irq_check(int irq, void *data)
{
	struct nvme_queue *nvmeq = data;

    pr_info("%s() irq[%d] (Ln %d)\n", __func__, irq, __LINE__);  // KW_DB
	if (nvme_cqe_pending(nvmeq))
		return IRQ_WAKE_THREAD;
	return IRQ_NONE;
}

/*
 * Poll for completions for any interrupt driven queue
 * Can be called from any context.
 */
static void nvme_poll_irqdisable(struct nvme_queue *nvmeq)
{
	struct pci_dev *pdev = to_pci_dev(nvmeq->dev->dev);

    pr_info("%s() Entered. (Ln %d)\n", __func__, __LINE__); // KW_DB_20241112
	WARN_ON_ONCE(test_bit(NVMEQ_POLLED, &nvmeq->flags));

	disable_irq(pci_irq_vector(pdev, nvmeq->cq_vector));
	nvme_poll_cq(nvmeq, NULL);
	enable_irq(pci_irq_vector(pdev, nvmeq->cq_vector));
}

static int nvme_poll(struct blk_mq_hw_ctx *hctx, struct io_comp_batch *iob)
{
	struct nvme_queue *nvmeq = hctx->driver_data;
	bool found;

	// pr_info("%s() Entered. (Ln %d)\n", __func__, __LINE__); // KW_DB_20241112    // KW_DB_20250116
    if (!nvme_cqe_pending(nvmeq))
		return 0;

    // KW_DB_20250408 - Enable the following for debugging.
    // pr_info("%s() cqe pending. (Ln %d)\n", __func__, __LINE__); // KW_DB_20250116
	spin_lock(&nvmeq->cq_poll_lock);
	found = nvme_poll_cq(nvmeq, iob);
	spin_unlock(&nvmeq->cq_poll_lock);

	return found;
}

static void nvme_pci_submit_async_event(struct nvme_ctrl *ctrl)
{
	struct nvme_dev *dev = to_nvme_dev(ctrl);
	struct nvme_queue *nvmeq = &dev->queues[0];
	struct nvme_command c = { };

	c.common.opcode = nvme_admin_async_event;
	c.common.command_id = NVME_AQ_BLK_MQ_DEPTH;

	spin_lock(&nvmeq->sq_lock);
	nvme_sq_copy_cmd(nvmeq, &c);
    pr_info("%s(): Calling nvme_write_sq_db(). opcode[0x%X] (Ln %d)\n", __func__, c.common.opcode, __LINE__);    // KW_DB
	nvme_write_sq_db(nvmeq, true);
	spin_unlock(&nvmeq->sq_lock);
}

static int adapter_delete_queue(struct nvme_dev *dev, u8 opcode, u16 id)
{
	struct nvme_command c = { };

	c.delete_queue.opcode = opcode;
	c.delete_queue.qid = cpu_to_le16(id);

	return nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
}

static int adapter_alloc_cq(struct nvme_dev *dev, u16 qid,
		struct nvme_queue *nvmeq, s16 vector)
{
	struct nvme_command c = { };
	int flags = NVME_QUEUE_PHYS_CONTIG;
    // KW_DB->
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    dma_addr_t      addr;
    u32             widx, addr32;
    u32             realign_addr_lo;
    u32             realign_offset;
    // KW_DB<-

	if (!test_bit(NVMEQ_POLLED, &nvmeq->flags))
		flags |= NVME_CQ_IRQ_ENABLED;

    ///dev_info(dev->ctrl.device, "%s() qid[%d]: cq_dma_addr[0x%llX] (Ln %d)\n", __func__, qid, nvmeq->cq_dma_addr, __LINE__); // KW_DB
// getFreeWidx
	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
// KW_DB->

#if 1
    // dev_info(dev->ctrl.device, "%s() qid[%d]: cq_dma_addr[0x%llX] (Ln %d)\n", __func__, qid, nvmeq->cq_dma_addr, __LINE__); /// KW_DB
    dev_info(dev->ctrl.device, "%s() qid[%d]: cq_dma_addr[0x%llX] vector[%d] (Ln %d)\n", __func__, qid, nvmeq->cq_dma_addr, vector, __LINE__); // KW_DB_20240611

// KW_DB_20250124->
#if (USE_MW_SQ_CQ)
    dev_info(dev->ctrl.device, "%s() qid[%d]: cq_dma_addr[0x%llX] vector[%d] (Ln %d)\n", __func__, qid, nvmeq->cq_dma_addr, vector, __LINE__); // KW_DB
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(dev->mw0_addr + nvmeq->cq_dma_addr);
#else
// KW_DB_20250124<-

    getFreeWidx(dev, &widx);
    dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n", __func__, widx, __LINE__); // KW_DB

    // Realign address based on NTB alignment requirement.
    //addr32 = (u32)(nvmeq->cq_dma_addr & 0xFFFFFFFF);
    realign_addr_lo = my_round_down(dev, (u32)(nvmeq->cq_dma_addr & 0xFFFFFFFF));
    realign_offset = ((u32)(nvmeq->cq_dma_addr & 0xFFFFFFFF)) - realign_addr_lo;
    addr32 = realign_addr_lo;

    addr32 |= qid;
    // Request reverse mapping with LUT.
    pci_write_config_dword(pdev, NTB_NVME_CQ_ADDR_LO, addr32);

    addr32 = (u32)(nvmeq->cq_dma_addr >> 32);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    addr32 |= ((u32) qid << ADDR_HI_QID_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_CQ_ADDR_HI, addr32);

    // Apply alignment offset.
    //addr = dev->dnb_addr + ((widx+1) * SZ_32K);
    addr = dev->dnb_addr + ((widx+1) * SZ_32K) + realign_offset;

	c.create_sq.opcode = nvme_admin_create_cq;
	c.create_sq.prp1 = cpu_to_le64(addr);
#endif  /* USE_MW_SQ_CQ */  // KW_DB_20250124
#else
// KW_DB<-
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(nvmeq->cq_dma_addr);
#endif

	c.create_cq.cqid = cpu_to_le16(qid);
	c.create_cq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(flags);
	c.create_cq.irq_vector = cpu_to_le16(vector);

	return nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
}

static int adapter_alloc_sq(struct nvme_dev *dev, u16 qid,
						struct nvme_queue *nvmeq)
{
	struct nvme_ctrl *ctrl = &dev->ctrl;
	struct nvme_command c = { };
	int flags = NVME_QUEUE_PHYS_CONTIG;
    // KW_DB->
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    dma_addr_t      addr;
    u32             widx, addr32;
    u32             realign_addr_lo;
    u32             realign_offset;
    // KW_DB<-

	/*
	 * Some drives have a bug that auto-enables WRRU if MEDIUM isn't
	 * set. Since URGENT priority is zeroes, it makes all queues
	 * URGENT.
	 */
	if (ctrl->quirks & NVME_QUIRK_MEDIUM_PRIO_SQ)
		flags |= NVME_SQ_PRIO_MEDIUM;

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
// KW_DB->
#if 1
    ///dev_info(dev->ctrl.device, "%s() qid[%d]: sq_dma_addr[0x%llX] (Ln %d)\n", __func__, nvmeq->qid, nvmeq->sq_dma_addr, __LINE__); // KW_DB
// KW_DB_20250124->
#if (USE_MW_SQ_CQ)
    dev_info(dev->ctrl.device, "%s() qid[%d]: sq_dma_addr[0x%llX] (Ln %d)\n", __func__, nvmeq->qid, nvmeq->sq_dma_addr, __LINE__); // KW_DB
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(dev->mw0_addr + nvmeq->sq_dma_addr);
#else
// KW_DB_20250124<-
    getFreeWidx(dev, &widx);
    dev_info(dev->ctrl.device, "%s() qid[%d]: widx[0x%X] allocated. (Ln %d)\n", __func__, qid, widx, __LINE__); // KW_DB

    // Realign address based on NTB alignment requirement.
    //addr32 = (u32)(nvmeq->sq_dma_addr & 0xFFFFFFFF);
    realign_addr_lo = my_round_down(dev, (u32)(nvmeq->sq_dma_addr & 0xFFFFFFFF));
    realign_offset = ((u32)(nvmeq->sq_dma_addr & 0xFFFFFFFF)) - realign_addr_lo;
    addr32 = realign_addr_lo;

    addr32 |= nvmeq->qid;
    // Request reverse mapping with LUT.
    pci_write_config_dword(pdev, NTB_NVME_SQ_ADDR_LO, addr32);

    addr32 = (u32)(nvmeq->sq_dma_addr >> 32);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    addr32 |= ((u32) nvmeq->qid << ADDR_HI_QID_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_SQ_ADDR_HI, addr32);

    // Apply alignment offset.
    //addr = dev->dnb_addr + ((widx+1) * SZ_32K);
    addr = dev->dnb_addr + ((widx+1) * SZ_32K) + realign_offset;

	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(addr);
#endif  /* USE_MW_SQ_CQ */  // KW_DB_20250124
#else
// KW_DB<-
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(nvmeq->sq_dma_addr);
#endif  // KW_DB
	c.create_sq.sqid = cpu_to_le16(qid);
	c.create_sq.qsize = cpu_to_le16(nvmeq->q_depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(flags);
	c.create_sq.cqid = cpu_to_le16(qid);

	return nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
}

static int adapter_delete_cq(struct nvme_dev *dev, u16 cqid)
{
	return adapter_delete_queue(dev, nvme_admin_delete_cq, cqid);
}

static int adapter_delete_sq(struct nvme_dev *dev, u16 sqid)
{
	return adapter_delete_queue(dev, nvme_admin_delete_sq, sqid);
}

static enum rq_end_io_ret abort_endio(struct request *req, blk_status_t error)
{
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;

	dev_warn(nvmeq->dev->ctrl.device,
		 "Abort status: 0x%x", nvme_req(req)->status);
	atomic_inc(&nvmeq->dev->ctrl.abort_limit);
	blk_mq_free_request(req);
	return RQ_END_IO_NONE;
}

static bool nvme_should_reset(struct nvme_dev *dev, u32 csts)
{
	/* If true, indicates loss of adapter communication, possibly by a
	 * NVMe Subsystem reset.
	 */
	bool nssro = dev->subsystem && (csts & NVME_CSTS_NSSRO);

	/* If there is a reset/reinit ongoing, we shouldn't reset again. */
	switch (dev->ctrl.state) {
	case NVME_CTRL_RESETTING:
	case NVME_CTRL_CONNECTING:
		return false;
	default:
		break;
	}

	/* We shouldn't reset unless the controller is on fatal error state
	 * _or_ if we lost the communication with it.
	 */
	if (!(csts & NVME_CSTS_CFS) && !nssro)
		return false;

	return true;
}

static void nvme_warn_reset(struct nvme_dev *dev, u32 csts)
{
	/* Read a config register to help see what died. */
	u16 pci_status;
	int result;

	result = pci_read_config_word(to_pci_dev(dev->dev), PCI_STATUS,
				      &pci_status);
	if (result == PCIBIOS_SUCCESSFUL)
		dev_warn(dev->ctrl.device,
			 "controller is down; will reset: CSTS=0x%x, PCI_STATUS=0x%hx\n",
			 csts, pci_status);
	else
		dev_warn(dev->ctrl.device,
			 "controller is down; will reset: CSTS=0x%x, PCI_STATUS read failed (%d)\n",
			 csts, result);

	if (csts != ~0)
		return;

	dev_warn(dev->ctrl.device,
		 "Does your device have a faulty power saving mode enabled?\n");
	dev_warn(dev->ctrl.device,
		 "Try \"nvme_core.default_ps_max_latency_us=0 pcie_aspm=off\" and report a bug\n");
}

static enum blk_eh_timer_return nvme_timeout(struct request *req)
{
	struct nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_queue *nvmeq = req->mq_hctx->driver_data;
	struct nvme_dev *dev = nvmeq->dev;
	struct request *abort_req;
	struct nvme_command cmd = { };
	u32 csts = readl(dev->bar + NVME_REG_CSTS);

    dev_warn(dev->ctrl.device, "%s() qid[%d]: csts[0x%X]. (Ln %d)\n",
        __func__, nvmeq->qid, csts, __LINE__); // KW_DB
	/* If PCI error recovery process is happening, we cannot reset or
	 * the recovery mechanism will surely fail.
	 */
	mb();
	if (pci_channel_offline(to_pci_dev(dev->dev)))
		return BLK_EH_RESET_TIMER;

	/*
	 * Reset immediately if the controller is failed
	 */
	if (nvme_should_reset(dev, csts)) {
		nvme_warn_reset(dev, csts);
		goto disable;
	}

	/*
	 * Did we miss an interrupt?
	 */
	if (test_bit(NVMEQ_POLLED, &nvmeq->flags))
		nvme_poll(req->mq_hctx, NULL);
	else
		nvme_poll_irqdisable(nvmeq);

	if (blk_mq_rq_state(req) != MQ_RQ_IN_FLIGHT) {
		dev_warn(dev->ctrl.device,
			 "I/O %d QID %d timeout, completion polled\n",
			 req->tag, nvmeq->qid);
		return BLK_EH_DONE;
	}

	/*
	 * Shutdown immediately if controller times out while starting. The
	 * reset work will see the pci device disabled when it gets the forced
	 * cancellation error. All outstanding requests are completed on
	 * shutdown, so we return BLK_EH_DONE.
	 */
	switch (dev->ctrl.state) {
	case NVME_CTRL_CONNECTING:
		nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
		fallthrough;
	case NVME_CTRL_DELETING:
		dev_warn_ratelimited(dev->ctrl.device,
			 "I/O %d QID %d timeout, disable controller\n",
			 req->tag, nvmeq->qid);
		nvme_req(req)->flags |= NVME_REQ_CANCELLED;
		nvme_dev_disable(dev, true);
		return BLK_EH_DONE;
	case NVME_CTRL_RESETTING:
		return BLK_EH_RESET_TIMER;
	default:
		break;
	}

	/*
	 * Shutdown the controller immediately and schedule a reset if the
	 * command was already aborted once before and still hasn't been
	 * returned to the driver, or if this is the admin queue.
	 */
	if (!nvmeq->qid || iod->aborted) {
		dev_warn(dev->ctrl.device,
			 "I/O %d QID %d timeout, reset controller\n",
			 req->tag, nvmeq->qid);
		nvme_req(req)->flags |= NVME_REQ_CANCELLED;
		goto disable;
	}

	if (atomic_dec_return(&dev->ctrl.abort_limit) < 0) {
		atomic_inc(&dev->ctrl.abort_limit);
		return BLK_EH_RESET_TIMER;
	}
	iod->aborted = true;

	cmd.abort.opcode = nvme_admin_abort_cmd;
	cmd.abort.cid = nvme_cid(req);
	cmd.abort.sqid = cpu_to_le16(nvmeq->qid);

	dev_warn(nvmeq->dev->ctrl.device,
		"I/O %d (%s) QID %d timeout, aborting\n",
		 req->tag,
		 nvme_get_opcode_str(nvme_req(req)->cmd->common.opcode),
		 nvmeq->qid);

	abort_req = blk_mq_alloc_request(dev->ctrl.admin_q, nvme_req_op(&cmd),
					 BLK_MQ_REQ_NOWAIT);
	if (IS_ERR(abort_req)) {
		atomic_inc(&dev->ctrl.abort_limit);
		return BLK_EH_RESET_TIMER;
	}
	nvme_init_request(abort_req, &cmd);

	abort_req->end_io = abort_endio;
	abort_req->end_io_data = NULL;
	blk_execute_rq_nowait(abort_req, false);

	/*
	 * The aborted req will be completed on receiving the abort req.
	 * We enable the timer again. If hit twice, it'll cause a device reset,
	 * as the device then is in a faulty state.
	 */
	return BLK_EH_RESET_TIMER;

disable:
	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_RESETTING))
		return BLK_EH_DONE;

	nvme_dev_disable(dev, false);
	if (nvme_try_sched_reset(&dev->ctrl))
		nvme_unquiesce_io_queues(&dev->ctrl);
	return BLK_EH_DONE;
}

static void nvme_free_queue(struct nvme_queue *nvmeq)
{
	dma_free_coherent(nvmeq->dev->dev, CQ_SIZE(nvmeq),
				(void *)nvmeq->cqes, nvmeq->cq_dma_addr);
	if (!nvmeq->sq_cmds)
		return;

	if (test_and_clear_bit(NVMEQ_SQ_CMB, &nvmeq->flags)) {
		pci_free_p2pmem(to_pci_dev(nvmeq->dev->dev),
				nvmeq->sq_cmds, SQ_SIZE(nvmeq));
	} else {
		dma_free_coherent(nvmeq->dev->dev, SQ_SIZE(nvmeq),
				nvmeq->sq_cmds, nvmeq->sq_dma_addr);
	}
}

static void nvme_free_queues(struct nvme_dev *dev, int lowest)
{
	int i;

    // KW_DB->
    u32 * ptr;
    if (lowest == 0)
    {
        struct nvme_queue *nvmeq = &dev->queues[lowest];
        if (nvmeq->sq_cmds)
        {
            ptr = (u32 *) nvmeq->sq_cmds;
            // pr_info("%s(): cmd dword00[0x%X]. (Ln %d)\n",__func__,  *ptr, __LINE__);
            for (i=0; i<16; i++)
            {
                pr_info("%s(): cmd dword%02d[0x%X]. (Ln %d)\n",__func__,  i, *ptr, __LINE__); // KW_DB
                ptr++;
            }
        }
        if (nvmeq->cqes)
        {
            pr_info("%s(): cqe status[0x%X]. (Ln %d)\n",__func__,  le16_to_cpu(nvmeq->cqes->status), __LINE__); // KW_DB
            ptr = (u32 *) nvmeq->cqes;
            for (i=0; i<4; i++)
            {
                pr_info("%s(): cqe dword%02d[0x%X]. (Ln %d)\n",__func__,  i, *ptr, __LINE__); // KW_DB
                ptr++;
            }
        }
#if (USE_DBUF)
        if (nvmeq->dBuf)
        {
            ptr = (u32 *) nvmeq->dBuf;
            for (i=0; i<16; i++)
            {
                pr_info("%s(): dBuf dword%02d[0x%X]. (Ln %d)\n",__func__,  i, *ptr, __LINE__); // KW_DB
                ptr++;
            }
            dma_free_coherent(nvmeq->dev->dev, 8192, nvmeq->dBuf, nvmeq->dBufDmaAddr);
        }
#endif
    }
    // KW_DB<-

	for (i = dev->ctrl.queue_count - 1; i >= lowest; i--) {
		dev->ctrl.queue_count--;
		nvme_free_queue(&dev->queues[i]);
	}
}

// KW_DB_20240828->
/**
 *
 */
static void clear_msix_alloc_tracker(struct nvme_dev *dev, u32 entry)
{
    dev->msix_widx[entry] = 0;
    dev->msix_vector_ldw[entry] = 0;
    dev->msix_vector_udw[entry] = 0;
}
// KW_DB_20240828<-

static void nvme_suspend_queue(struct nvme_dev *dev, unsigned int qid)
{
	struct nvme_queue *nvmeq = &dev->queues[qid];

	if (!test_and_clear_bit(NVMEQ_ENABLED, &nvmeq->flags))
		return;

	/* ensure that nvme_queue_rq() sees NVMEQ_ENABLED cleared */
	mb();

	nvmeq->dev->online_queues--;
	if (!nvmeq->qid && nvmeq->dev->ctrl.admin_q)
		nvme_quiesce_admin_queue(&nvmeq->dev->ctrl);
	if (!test_and_clear_bit(NVMEQ_POLLED, &nvmeq->flags))
    {
        dev_info(dev->ctrl.device, "%s() Calling pci_free_irq() (Ln %d)\n", __func__, __LINE__); // KW_DB
		pci_free_irq(to_pci_dev(dev->dev), nvmeq->cq_vector, nvmeq);
        clear_msix_alloc_tracker(dev, qid);   // KW_DB_20240828
    }
}

static void nvme_suspend_io_queues(struct nvme_dev *dev)
{
	int i;

	for (i = dev->ctrl.queue_count - 1; i > 0; i--)
		nvme_suspend_queue(dev, i);
}

/*
 * Called only on a device that has been disabled and after all other threads
 * that can check this device's completion queues have synced, except
 * nvme_poll(). This is the last chance for the driver to see a natural
 * completion before nvme_cancel_request() terminates all incomplete requests.
 */
static void nvme_reap_pending_cqes(struct nvme_dev *dev)
{
	int i;

	pr_info("%s() Entered. (Ln %d)\n", __func__, __LINE__); // KW_DB_20241112
    for (i = dev->ctrl.queue_count - 1; i > 0; i--) {
		spin_lock(&dev->queues[i].cq_poll_lock);
		nvme_poll_cq(&dev->queues[i], NULL);
		spin_unlock(&dev->queues[i].cq_poll_lock);
	}
}

static int nvme_cmb_qdepth(struct nvme_dev *dev, int nr_io_queues,
				int entry_size)
{
	int q_depth = dev->q_depth;
	unsigned q_size_aligned = roundup(q_depth * entry_size,
					  NVME_CTRL_PAGE_SIZE);

	if (q_size_aligned * nr_io_queues > dev->cmb_size) {
		u64 mem_per_q = div_u64(dev->cmb_size, nr_io_queues);

		mem_per_q = round_down(mem_per_q, NVME_CTRL_PAGE_SIZE);
		q_depth = div_u64(mem_per_q, entry_size);

		/*
		 * Ensure the reduced q_depth is above some threshold where it
		 * would be better to map queues in system memory with the
		 * original depth
		 */
		if (q_depth < 64)
			return -ENOMEM;
	}

	return q_depth;
}

static int nvme_alloc_sq_cmds(struct nvme_dev *dev, struct nvme_queue *nvmeq,
				int qid)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);

    ///dev_info(dev->ctrl.device, "%s() Entered qid[%d] (Ln %d)\n", __func__, qid, __LINE__); // KW_DB

	if (qid && dev->cmb_use_sqes && (dev->cmbsz & NVME_CMBSZ_SQS)) {
		nvmeq->sq_cmds = pci_alloc_p2pmem(pdev, SQ_SIZE(nvmeq));
		if (nvmeq->sq_cmds) {
			nvmeq->sq_dma_addr = pci_p2pmem_virt_to_bus(pdev,
							nvmeq->sq_cmds);
			if (nvmeq->sq_dma_addr) {
				set_bit(NVMEQ_SQ_CMB, &nvmeq->flags);
				return 0;
			}
            dev_info(dev->ctrl.device, "%s() qid[%d] sq_dma_addr[0x%llX] (Ln %d)\n", __func__, qid, nvmeq->sq_dma_addr, __LINE__); // KW_DB

			pci_free_p2pmem(pdev, nvmeq->sq_cmds, SQ_SIZE(nvmeq));
		}
	}


	nvmeq->sq_cmds = dma_alloc_coherent(dev->dev, SQ_SIZE(nvmeq),
				&nvmeq->sq_dma_addr, GFP_KERNEL);

	if (!nvmeq->sq_cmds)
		return -ENOMEM;
    dev_info(dev->ctrl.device, "%s() qid[%d] sq_dma_addr[0x%llX] (Ln %d)\n", __func__, qid, nvmeq->sq_dma_addr, __LINE__); // KW_DB

    // KW_DB->
#if (USE_DBUF)
    if (qid == 0)
    {
        nvmeq->dBuf = dma_alloc_coherent(dev->dev, 8192, &nvmeq->dBufDmaAddr, GFP_KERNEL);
        if (nvmeq->dBuf)
            dev_info(dev->ctrl.device, "%s() sq_dma_addr[0x%llX] (Ln %d)\n", __func__, nvmeq->dBufDmaAddr, __LINE__); // KW_DB
    }
#endif
    // KW_DB<-

	return 0;
}

// KW_DB->
#if 0
/**
 * Update SQ and CQ size based on NTB LUT size alignment requirement.
 */
static void update_queues_size(struct nvme_dev *dev, struct nvme_queue *nvmeq)
{
    nvmeq->sq_size_orig = nvmeq->q_depth * nvmeq->sqes;
    nvmeq->cq_size_orig = nvmeq->q_depth * sizeof(struct nvme_completion);
    nvmeq->sq_size_new = round_up(nvmeq->sq_size_orig, dev->lut_size_align);
    nvmeq->cq_size_new = round_up(nvmeq->cq_size_orig, dev->lut_size_align);

    dev_info(dev->ctrl.device, "%s() sq_size [0x%lX]=>[0x%lX] (Ln %d)\n", __func__, nvmeq->sq_size_orig, nvmeq->sq_size_new, __LINE__); // KW_DB
    dev_info(dev->ctrl.device, "%s() cq_size [0x%lX]=>[0x%lX] (Ln %d)\n", __func__, nvmeq->cq_size_orig, nvmeq->cq_size_new, __LINE__); // KW_DB
}


// TODO: Add code:
	if (!IS_ALIGNED(peer->pciCfgBuf_xlat, xlat_align)) {
		dev_err(&perf->ntb->dev, "Unaligned pciCfgBuf allocated\n");
		goto err_free_pciCfgBuf;
	}

 #endif


/**
 *
 */

void __iomem * map_msix_table(struct nvme_dev *dev, u32 table_offset, u16 length)
{
    struct pci_dev  *pdev = to_pci_dev(dev->dev);
    void __iomem    *msix_table = NULL;

    msix_table = ioremap(pci_resource_start(pdev, 0) + table_offset, length);

    return msix_table;
}

/**
 *
 */

void __iomem * map_msix_table2(struct nvme_dev *dev)
{
    struct pci_dev  *pdev = to_pci_dev(dev->dev);
    void __iomem    *msix_table = NULL;
    u8              msix_pos;
    u8              msix_bar;
    u32             msix_table_offset;
    u16             msix_table_size;

    msix_pos = pdev->msix_cap;
    if (msix_pos)
    {
        u16 flags;
        u32 table;

        pci_read_config_word(pdev, msix_pos + PCI_MSIX_FLAGS, &flags);
        pci_read_config_dword(pdev, msix_pos + PCI_MSIX_TABLE, &table);

        msix_bar = table & PCI_MSIX_TABLE_BIR;
        msix_table_offset = table & PCI_MSIX_TABLE_OFFSET;
        msix_table_size = ((flags & PCI_MSIX_FLAGS_QSIZE) + 1) * 16;


        msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
    }

    return msix_table;
}

#define ULTRA

#ifndef ULTRA
/**
 *
 */
static void read_msix_vector(struct nvme_dev *dev, int idx, u32 *dword)
{
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    void __iomem   *msix_table = NULL;
    u8              msix_pos;
    u8              msix_bar;
    u32             msix_table_offset;
    u16             msix_table_size;

    msix_pos = pdev->msix_cap;
    if (msix_pos)
    {
        u16 flags;
        u32 table;

        pci_read_config_word(pdev, msix_pos + PCI_MSIX_FLAGS, &flags);
        pci_read_config_dword(pdev, msix_pos + PCI_MSIX_TABLE, &table);

        msix_bar = table & PCI_MSIX_TABLE_BIR;
        msix_table_offset = table & PCI_MSIX_TABLE_OFFSET;
        msix_table_size = ((flags & PCI_MSIX_FLAGS_QSIZE) + 1) * 16;

        dev_err(dev->ctrl.device, "%s() msix_table_offset[0x%X] msix_table_size[0x%X] (Ln %d)\n", __func__, msix_table_offset, msix_table_size, __LINE__); // KW_DB

        msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
        if (msix_table)
        {
            int i;

            for (i=0; i<4; i++)
            {
                dword[i] = readl(msix_table+(i*4) + (4*idx));
                #if 0   // Enable for debugging
                dev_info(dev->ctrl.device, "%s() irqVector+%04d[0x%X] (Ln %d)\n",
                    __func__, (i*4)+(4*idx), dword[i], __LINE__);
                #endif
            }

            iounmap(msix_table);
        }
        else
        {
            dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
        }
    }
}
#endif  /* ULTRA */


#ifdef ULTRA
/**
 *
 */
static void dump_msix_vectors(struct nvme_dev *dev, u32 count)
{
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    void __iomem   *msix_table = NULL;
    u8              msix_pos;
    u8              msix_bar;
    u32             msix_table_offset;
    u16             msix_table_size;
    u32             dword[4];

    msix_pos = pdev->msix_cap;
    if (msix_pos)
    {
        u16 flags;
        u32 table;

        pci_read_config_word(pdev, msix_pos + PCI_MSIX_FLAGS, &flags);
        pci_read_config_dword(pdev, msix_pos + PCI_MSIX_TABLE, &table);

        msix_bar = table & PCI_MSIX_TABLE_BIR;
        msix_table_offset = table & PCI_MSIX_TABLE_OFFSET;
        msix_table_size = ((flags & PCI_MSIX_FLAGS_QSIZE) + 1) * 16;

        dev_err(dev->ctrl.device, "%s() msix_table_offset[0x%X] msix_table_size[0x%X] count[%d] (Ln %d)\n",
            __func__, msix_table_offset, msix_table_size, count, __LINE__); // KW_DB

        msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
        if (msix_table)
        {
            int i, entry;

            for (entry=0; entry<count; entry++)
            {
                for (i=0; i<4; i++)
                {
                    dword[i] = readl(msix_table+(i*4) + (16*entry));
                    #if 0   // Enable for debugging
                    dev_info(dev->ctrl.device, "%s() irqVector+%04d[0x%X] (Ln %d)\n",
                        __func__, (i*4)+(4*idx), dword[i], __LINE__);
                    #endif
                }
                dev_info(dev->ctrl.device, "%s() #%02d: DW3[0x%X] DW2[0x%X] DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                    __func__, entry, dword[3], dword[2], dword[1], dword[0], __LINE__);
            }

            iounmap(msix_table);
        }
        else
        {
            dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
        }
    }
}

/**
 *
 */
static void update_msix_table_entry(struct nvme_dev *dev, void __iomem   *msix_table, u32 msix_entry[], u32 entry)
{
    u32 realign_addr_lo;
    u32 offset;
    u32 addr32;
    dma_addr_t addr64;
    u32 widx = dev->msix_widx[entry]; ;
    u32 ctrlRegVal;
    bool unmasked;
        
        
    // Request reverse mapping with LUT.
    realign_addr_lo = my_round_down(dev, msix_entry[0]);
    offset = msix_entry[0] - realign_addr_lo;

    pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO, realign_addr_lo);
    addr32 = msix_entry[1] | (widx << ADDR_HI_WIDX_SHIFT);
    pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI, addr32);

    ctrlRegVal = readl(msix_table + 12); /* Read control register. */
    unmasked = !(ctrlRegVal & PCI_MSIX_ENTRY_CTRL_MASKBIT);
    
    if (unmasked)
        writel(ctrlRegVal | PCI_MSIX_ENTRY_CTRL_MASKBIT, msix_table + 12);

    // Write LUT addresses to MSI-X table.
    addr64 = dev->dnb_addr + ((widx+1) * SZ_32K) + offset;

    lo_hi_writeq(addr64, msix_table);


    dev_info(dev->ctrl.device, "%s() Wrote addr64[0x%llX] to MSI-X table entry %02d unmasked[%d] (Ln %d)\n",
        __func__, addr64, entry, unmasked, __LINE__); // KW_DB

    if (unmasked)
        writel(ctrlRegVal , msix_table + 12);
            
    dev_info(dev->ctrl.device, "%s() Updating msix_vector_ldw[[%02d] from [0x%X] to [0x%X] (Ln %d)\n",
        __func__, entry, dev->msix_vector_ldw[entry], (u32) (addr64 & 0xFFFFFFFF), __LINE__); // KW_DB_20240828
    dev_info(dev->ctrl.device, "%s() Updating msix_vector_udw[[%02d] from [0x%X] to [0x%X] (Ln %d)\n",
        __func__, entry, dev->msix_vector_udw[entry], (u32) (addr64 >> 32), __LINE__); // KW_DB_20240828
    dev->msix_vector_ldw[entry] = (u32) (addr64 & 0xFFFFFFFF);
    dev->msix_vector_udw[entry] = (u32) (addr64 >> 32);
}

#define minVal(a, b) ((a) < (b) ? (a) : (b))
/**
 *
 */
static void dump_msix_table_entry(struct nvme_dev *dev, u32 entry, u32 entryCount, u32 caller)
{
#if 1   // KW_DB - '1' to enable.
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    void __iomem   *msix_table = NULL;
    u8              msix_pos;
    // u8              msix_bar;
    u32             msix_table_offset;
    u16             msix_table_size;
    u32             dword[4];
    int             cnt = minVal(4, entryCount);

    if (cnt < 1)
    {
        dev_err(dev->ctrl.device, "%s() Invalid entryCount[%d]!\n", __func__, entryCount);
        return;
    }
    msix_pos = pdev->msix_cap;
    if (msix_pos)
    {
        u16 flags;
        u32 table;

        pci_read_config_word(pdev, msix_pos + PCI_MSIX_FLAGS, &flags);
        pci_read_config_dword(pdev, msix_pos + PCI_MSIX_TABLE, &table);

        // msix_bar = table & PCI_MSIX_TABLE_BIR;
        msix_table_offset = table & PCI_MSIX_TABLE_OFFSET;
        msix_table_size = ((flags & PCI_MSIX_FLAGS_QSIZE) + 1) * 16;

        ///dev_err(dev->ctrl.device, "%s() msix_table_offset[0x%X] msix_table_size[0x%X] entry[%d] (Ln %d)\n",
        ///    __func__, msix_table_offset, msix_table_size, entry, __LINE__); // KW_DB

        msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
        if (msix_table)
        {
            int i;

            #if 0  // Enable for 1 DW only.
            dword[0] = readl(msix_table + (16*entry));
            dev_info(dev->ctrl.device, "%s() #%02d: DW0[0x%X] (Ln %d)\n",
                __func__, entry, dword[0], caller);
            #else
            for (i=0; i<cnt; i++)
            {
                dword[i] = readl(msix_table+(i*4) + (16*entry));
                #if 0   // Enable for debugging
                dev_info(dev->ctrl.device, "%s() irqVector+%04d[0x%X] (Ln %d)\n",
                    __func__, (i*4)+(4*idx), dword[i], __LINE__);
                #endif
            }
            switch (cnt)
            {
                case 1:
                    dev_info(dev->ctrl.device, "%s() #%02d: DW0[0x%X] (Ln %d)\n",
                        __func__, entry, dword[0], caller);
                        break;
                case 2:
                    dev_info(dev->ctrl.device, "%s() #%02d: DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                        __func__, entry, dword[1], dword[0], caller);
                        break;
                case 3:
                    dev_info(dev->ctrl.device, "%s() #%02d: DW2[0x%X] DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                        __func__, entry, dword[2], dword[1], dword[0], caller);
                        break;
                case 4:
                    dev_info(dev->ctrl.device, "%s() #%02d: DW3[0x%X] DW2[0x%X] DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                        __func__, entry, dword[3], dword[2], dword[1], dword[0], caller);
                        break;
                default:
                    dev_err(dev->ctrl.device, "%s() Invalid entryCount[%d]!\n", __func__, entryCount);

            }
            #endif

            ///dev_info(dev->ctrl.device, "%s() On file: #%02d: wide[%d] DW0[0x%X] (Ln %d)\n",
            ///            __func__, entry, dev->msix_widx[entry], dev->msix_vector_ldw[entry], caller);
            if (dev->msix_widx[entry] && (dword[0] != dev->msix_vector_ldw[entry]))
            {
                dev_err(dev->ctrl.device, "%s() Missmatch! Update msix_table entry[%d] (Ln %d)\n", __func__, entry, __LINE__); // KW_DB
                update_msix_table_entry(dev, msix_table, dword, entry);
            }


            iounmap(msix_table);
        }
        else
        {
            dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
        }
    }
#endif
}

#if 0   // KW_DB_20240828
/**
 *
 */
static void rewrite_msix_table(struct nvme_dev *dev, u32 count)
{
    void __iomem   *msix_table = NULL;
    u32 dword[4];
    u32 realign_addr_lo;
    u32 offset;
    u32 addr32;
    u32 widx;
    int entry, i;
    dma_addr_t addr;

    msix_table = map_msix_table2(dev);
    if (msix_table)
    {
        ///dev_info(dev->ctrl.device, "%s(): Original MSI-X settings: (Ln %d)\n", __func__, __LINE__);
        for (entry=0; entry<count; entry++)
        {
            u32 ctrlRegVal;
            bool unmasked;
        
            ///read_msix_vector(dev, cnt, dword);
            for (i=0; i<4; i++)
            {
                dword[i] = readl(msix_table+(i*4) + (16*entry));
                #if 0   // Enable for debugging
                dev_info(dev->ctrl.device, "%s() irqVector+%04d[0x%X] (Ln %d)\n",
                    __func__, (i*4)+(4*idx), dword[i], __LINE__);
                #endif
            }

            // for (i=0; i<4; i++)
            // {
                // dev_info(dev->ctrl.device, "%s() msi-x %d dword+%02d[0x%X] (Ln %d)\n", __func__, entry, i, dword[i], __LINE__);
            // }
            dev_info(dev->ctrl.device, "%s() #%02d: DW3[0x%X] DW2[0x%X] DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                    __func__, entry, dword[3], dword[2], dword[1], dword[0], __LINE__);

            getFreeWidx(dev, &widx);
            dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n", __func__, widx, __LINE__);

            // Request reverse mapping with LUT.
            realign_addr_lo = my_round_down(dev, dword[0]);
            offset = dword[0] - realign_addr_lo;

            pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO, realign_addr_lo);
            addr32 = dword[1] | (widx << ADDR_HI_WIDX_SHIFT);
            pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI, addr32);

            ctrlRegVal = dword[3]; /* Read control register. */
            unmasked = !(ctrlRegVal & PCI_MSIX_ENTRY_CTRL_MASKBIT);
            
            if (unmasked)
                writel(ctrlRegVal | PCI_MSIX_ENTRY_CTRL_MASKBIT, msix_table + (16*entry) + 12);
        
            // Write LUT addresses to MSI-X table.
            addr = dev->dnb_addr + ((widx+1) * SZ_32K) + offset;

            lo_hi_writeq(addr, msix_table);
            dev_info(dev->ctrl.device, "%s() Wrote addr[0x%llX] to MSI-X table entry %02d unmasked[%d] (Ln %d)\n",
                __func__, addr, entry, unmasked, __LINE__); // KW_DB

            if (unmasked)
                writel(ctrlRegVal , msix_table + (16*entry) + 12);
        }

        iounmap(msix_table);
    }
    else
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
}


/**
 * Clear MSI-X table entries.
 */
static void dump_msix_table_entries(struct nvme_dev *dev, u32 num_entries)
{
    void __iomem   *msix_table = NULL;
    u32 dword[4];
    int i, entry;

    msix_table = map_msix_table2(dev);
    if (msix_table)
    {
        for (entry=0; entry<num_entries; entry++)
        {
            for (i=0; i<4; i++)
            {
                dword[i] = readl(msix_table+(i*4) + (16*entry));
            }
            dev_info(dev->ctrl.device, "%s() #%02d: DW3[0x%X] DW2[0x%X] DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                    __func__, entry, dword[3], dword[2], dword[1], dword[0], __LINE__);
        }
        iounmap(msix_table);
    }
    else
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
}

/**
 * Clear MSI-X table entries.
 */
static void clear_msix_table_entries(struct nvme_dev *dev, u32 num_entries)
{
    void __iomem   *msix_table = NULL;
    int i, entry;

    msix_table = map_msix_table2(dev);
    if (msix_table)
    {
        for (entry=0; entry<num_entries; entry++)
        {
            for (i=0; i<4; i++)
            {
                writel(0, msix_table+(i*4) + (16*entry));
            }
        }
        iounmap(msix_table);
        dev_info(dev->ctrl.device, "%s() Cleared (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
    else
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
}
#endif  // KW_DB_20240828

/**
 *
 */
static void rewrite_msix_table_entry(struct nvme_dev *dev, u32 entry)
{
    void __iomem   *msix_table = NULL;
    u32 dword[4];
    u32 realign_addr_lo;
    u32 offset;
    u32 addr32;
    u32 widx;
    int i;
    dma_addr_t addr;

    msix_table = map_msix_table2(dev);
    if (msix_table)
    {
        u32 ctrlRegVal;
        bool unmasked;
        
        for (i=0; i<4; i++)
        {
            dword[i] = readl(msix_table+(i*4) + (16*entry));
            #if 0   // Enable for debugging
            dev_info(dev->ctrl.device, "%s() irqVector+%04d[0x%X] (Ln %d)\n",
                __func__, (i*4)+(4*idx), dword[i], __LINE__);
            #endif
        }

        dev_info(dev->ctrl.device, "%s() #%02d: DW3[0x%X] DW2[0x%X] DW1[0x%X] DW0[0x%X] (Ln %d)\n",
                __func__, entry, dword[3], dword[2], dword[1], dword[0], __LINE__);

        getFreeWidx(dev, &widx);
        dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n", __func__, widx, __LINE__);

        // Request reverse mapping with LUT.
        realign_addr_lo = my_round_down(dev, dword[0]);
        offset = dword[0] - realign_addr_lo;

        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO + (16*entry), realign_addr_lo);
        addr32 = dword[1] | (widx << ADDR_HI_WIDX_SHIFT);
        // KW_DB_20240815->
        // pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI, addr32);
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI + (16*entry), addr32);
        // KW_DB_20240815<-

        ctrlRegVal = dword[3]; /* Read control register. */
        unmasked = !(ctrlRegVal & PCI_MSIX_ENTRY_CTRL_MASKBIT);
        
        if (unmasked)
            writel(ctrlRegVal | PCI_MSIX_ENTRY_CTRL_MASKBIT, msix_table + (16*entry) + 12);

        // Write LUT addresses to MSI-X table.
        addr = dev->dnb_addr + ((widx+1) * SZ_32K) + offset;

        //lo_hi_writeq(addr, msix_table);
        lo_hi_writeq(addr, msix_table + (16*entry));
        dev_info(dev->ctrl.device, "%s() Wrote addr[0x%llX] to MSI-X table entry %02d unmasked[%d] (Ln %d)\n",
            __func__, addr, entry, unmasked, __LINE__); // KW_DB

        if (unmasked)
            writel(ctrlRegVal , msix_table + (16*entry) + 12);
        
        dev->msix_widx[entry] = widx;
        dev->msix_vector_ldw[entry] = (u32) (addr & 0xFFFFFFFF);
        dev->msix_vector_udw[entry] = (u32) (addr >> 32);
        iounmap(msix_table);
    }
    else
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
}
#endif  /* ULTRA */
// KW_DB<-

static int nvme_alloc_queue(struct nvme_dev *dev, int qid, int depth)
{
	struct nvme_queue *nvmeq = &dev->queues[qid];

	if (dev->ctrl.queue_count > qid)
		return 0;

    ///dev_info(dev->ctrl.device, "%s() qid[0x%X] (Ln %d)\n",  __func__, qid, __LINE__);


	nvmeq->sqes = qid ? dev->io_sqes : NVME_ADM_SQES;
	nvmeq->q_depth = depth;
    // KW_DB->
    if (qid < 2)
    {
        update_queues_size(dev, nvmeq);
    }
    // KW_DB<-
	nvmeq->cqes = dma_alloc_coherent(dev->dev, CQ_SIZE(nvmeq),
					 &nvmeq->cq_dma_addr, GFP_KERNEL);
	if (!nvmeq->cqes)
		goto free_nvmeq;

    dev_info(dev->ctrl.device, "%s() qid[%d] cq_dma_addr[0x%llX] depth[%d] (Ln %d)\n", __func__, qid, nvmeq->cq_dma_addr, depth, __LINE__); // KW_DB

	if (nvme_alloc_sq_cmds(dev, nvmeq, qid))
		goto free_cqdma;

	nvmeq->dev = dev;
	spin_lock_init(&nvmeq->sq_lock);
	spin_lock_init(&nvmeq->cq_poll_lock);
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_db = &dev->dbs[qid * 2 * dev->db_stride];
	nvmeq->qid = qid;
	dev->ctrl.queue_count++;

	return 0;

 free_cqdma:
	dma_free_coherent(dev->dev, CQ_SIZE(nvmeq), (void *)nvmeq->cqes,
			  nvmeq->cq_dma_addr);
 free_nvmeq:
	return -ENOMEM;
}

//static int queue_request_irq(struct nvme_queue *nvmeq)            // KW_DB
static int queue_request_irq(struct nvme_queue *nvmeq, u32 caller)  // KW_DB
{
	struct pci_dev *pdev = to_pci_dev(nvmeq->dev->dev);
	int nr = nvmeq->dev->ctrl.instance;


	if (use_threaded_interrupts) {
        pr_info("%s() Calling pci_request_irq() cpu[%d] qid[%d] (Ln %d)\n", __func__, current->on_cpu, nvmeq->qid, caller); // KW_DB
		return pci_request_irq(pdev, nvmeq->cq_vector, nvme_irq_check,
				nvme_irq, nvmeq, "nvme%dq%d", nr, nvmeq->qid);
	} else {
        pr_info("%s() qid[%d] vec[%d](Ln %d)\n", __func__, nvmeq->qid, nvmeq->cq_vector, caller); // KW_DB
		return pci_request_irq(pdev, nvmeq->cq_vector, nvme_irq,
				NULL, nvmeq, "nvme%dq%d", nr, nvmeq->qid);
	}
}

static void nvme_init_queue(struct nvme_queue *nvmeq, u16 qid)
{
	struct nvme_dev *dev = nvmeq->dev;

	nvmeq->sq_tail = 0;
	nvmeq->last_sq_tail = 0;
	nvmeq->cq_head = 0;
	nvmeq->cq_phase = 1;
	nvmeq->q_db = &dev->dbs[qid * 2 * dev->db_stride];
	memset((void *)nvmeq->cqes, 0, CQ_SIZE(nvmeq));
	nvme_dbbuf_init(dev, nvmeq, qid);
	dev->online_queues++;
	wmb(); /* ensure the first interrupt sees the initialization */
}

/*
 * Try getting shutdown_lock while setting up IO queues.
 */
static int nvme_setup_io_queues_trylock(struct nvme_dev *dev)
{
	/*
	 * Give up if the lock is being held by nvme_dev_disable.
	 */
	if (!mutex_trylock(&dev->shutdown_lock))
    {   // KW_DB
        dev_err(dev->ctrl.device, "%s() -ENODEV (Ln %d)\n", __func__, __LINE__); // KW_DB
		return -ENODEV;
    }   // KW_DB

	/*
	 * Controller is in wrong state, fail early.
	 */
	if (dev->ctrl.state != NVME_CTRL_CONNECTING) {
		mutex_unlock(&dev->shutdown_lock);
        dev_err(dev->ctrl.device, "%s() -ENODEV (Ln %d)\n", __func__, __LINE__); // KW_DB
		return -ENODEV;
	}

	return 0;
}

static int nvme_create_queue(struct nvme_queue *nvmeq, int qid, bool polled)
{
	struct nvme_dev *dev = nvmeq->dev;
	int result;
	u16 vector = 0;

	clear_bit(NVMEQ_DELETE_ERROR, &nvmeq->flags);

	/*
	 * A queue's vector matches the queue identifier unless the controller
	 * has only one vector available.
	 */
	if (!polled)
		vector = dev->num_vecs == 1 ? 0 : qid;
	else
		set_bit(NVMEQ_POLLED, &nvmeq->flags);

    dump_msix_table_entry(dev, 0, 4, __LINE__);  // KW_DB
	result = adapter_alloc_cq(dev, qid, nvmeq, vector);
	if (result)
		return result;

    dump_msix_table_entry(dev, 0, 4, __LINE__);  // KW_DB
    dump_msix_table_entry(dev, 1, 4, __LINE__);  // KW_DB
    ///dump_msix_table_entry(dev, 4, 4, __LINE__);  // KW_DB
    ///dump_msix_table_entry(dev, 5, 4, __LINE__);  // KW_DB
	result = adapter_alloc_sq(dev, qid, nvmeq);
	if (result < 0)
		return result;
	if (result)
		goto release_cq;

	nvmeq->cq_vector = vector;

	dump_msix_table_entry(dev, 0, 4, __LINE__);  // KW_DB
    result = nvme_setup_io_queues_trylock(dev);
	if (result)
		return result;
    dump_msix_table_entry(dev, 0, 4, __LINE__);  // KW_DB
	nvme_init_queue(nvmeq, qid);
	if (!polled) {
		//result = queue_request_irq(nvmeq);            // KW_DB
        result = queue_request_irq(nvmeq, __LINE__);    // KW_DB
		if (result < 0)
			goto release_sq;
	}
    dump_msix_table_entry(dev, 0, 4, __LINE__);  // KW_DB

#ifdef ULTRA
    dump_msix_table_entry(dev, qid, 4, __LINE__);
    if (qid < dev->nr_irq_queues)   // KW_DB_20250116
    {                               // KW_DB_20250116
        rewrite_msix_table_entry(dev, qid);
        dump_msix_table_entry(dev, 0, 4, __LINE__);  // KW_DB
    }                               // KW_DB_20250116
#endif

	set_bit(NVMEQ_ENABLED, &nvmeq->flags);
	mutex_unlock(&dev->shutdown_lock);
	return result;

release_sq:
	dev->online_queues--;
	mutex_unlock(&dev->shutdown_lock);
	adapter_delete_sq(dev, qid);
release_cq:
	adapter_delete_cq(dev, qid);
	return result;
}

static const struct blk_mq_ops nvme_mq_admin_ops = {
	.queue_rq	= nvme_queue_rq,
	.complete	= nvme_pci_complete_rq,
	.init_hctx	= nvme_admin_init_hctx,
	.init_request	= nvme_pci_init_request,
	.timeout	= nvme_timeout,
};

static const struct blk_mq_ops nvme_mq_ops = {
	.queue_rq	= nvme_queue_rq,
	.queue_rqs	= nvme_queue_rqs,
	.complete	= nvme_pci_complete_rq,
	.commit_rqs	= nvme_commit_rqs,
	.init_hctx	= nvme_init_hctx,
	.init_request	= nvme_pci_init_request,
	.map_queues	= nvme_pci_map_queues,
	.timeout	= nvme_timeout,
	.poll		= nvme_poll,
};

static void nvme_dev_remove_admin(struct nvme_dev *dev)
{
	if (dev->ctrl.admin_q && !blk_queue_dying(dev->ctrl.admin_q)) {
		/*
		 * If the controller was reset during removal, it's possible
		 * user requests may be waiting on a stopped queue. Start the
		 * queue to flush these to completion.
		 */
		nvme_unquiesce_admin_queue(&dev->ctrl);
		nvme_remove_admin_tag_set(&dev->ctrl);
	}
}

static unsigned long db_bar_size(struct nvme_dev *dev, unsigned nr_io_queues)
{
	return NVME_REG_DBS + ((nr_io_queues + 1) * 8 * dev->db_stride);
}

static int nvme_remap_bar(struct nvme_dev *dev, unsigned long size)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);

    dev_info(dev->ctrl.device, "%s() BAR size requested[0x%lX] (Ln %d)\n", __func__, size, __LINE__); // KW_DB
	if (size <= dev->bar_mapped_size)
		return 0;
	if (size > pci_resource_len(pdev, 0))
		return -ENOMEM;
	if (dev->bar)
		iounmap(dev->bar);
	dev->bar = ioremap(pci_resource_start(pdev, 0), size);
	if (!dev->bar) {
		dev->bar_mapped_size = 0;
		return -ENOMEM;
	}
	dev->bar_mapped_size = size;
	dev->dbs = dev->bar + NVME_REG_DBS;

	return 0;
}

static int nvme_pci_configure_admin_queue(struct nvme_dev *dev)
{
	int result;
    u32 addr32, widx; // KW_DB
	u32 aqa;
	struct nvme_queue *nvmeq;
    struct pci_dev *pdev = to_pci_dev(dev->dev); // KW_DB
    dma_addr_t addr; // KW_DB

    dev_info(dev->ctrl.device, "%s() Calling nvme_remap_bar() with size[0x%lX] (Ln %d)\n",
        __func__, (unsigned long) 0, __LINE__); // KW_DB
	result = nvme_remap_bar(dev, db_bar_size(dev, 0));
	if (result < 0)
		return result;

	dev->subsystem = readl(dev->bar + NVME_REG_VS) >= NVME_VS(1, 1, 0) ?
				NVME_CAP_NSSRC(dev->ctrl.cap) : 0;

	if (dev->subsystem &&
	    (readl(dev->bar + NVME_REG_CSTS) & NVME_CSTS_NSSRO))
		writel(NVME_CSTS_NSSRO, dev->bar + NVME_REG_CSTS);

	/*
	 * If the device has been passed off to us in an enabled state, just
	 * clear the enabled bit.  The spec says we should set the 'shutdown
	 * notification bits', but doing so may cause the device to complete
	 * commands to the admin queue ... and we don't know what memory that
	 * might be pointing at!
	 */
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	result = nvme_disable_ctrl(&dev->ctrl, false);
	if (result < 0)
    {
        dev_info(dev->ctrl.device, "%s() result[%d] (Ln %d)\n", __func__, result, __LINE__); // KW_DB
		return result;
    }
    read_msix_cap(dev, __func__, __LINE__); // KW_DB

	result = nvme_alloc_queue(dev, 0, NVME_AQ_DEPTH);
    
    read_msix_cap(dev, __func__, __LINE__); // KW_DB

	if (result)
    {
        dev_info(dev->ctrl.device, "%s() result[%d] (Ln %d)\n", __func__, result, __LINE__); // KW_DB
		return result;
    }

	dev->ctrl.numa_node = dev_to_node(dev->dev);

	nvmeq = &dev->queues[0];
	aqa = nvmeq->q_depth - 1;
	aqa |= aqa << 16;

    writel(aqa, dev->bar + NVME_REG_AQA);

// KW_DB->
#if 1
    mdelay(1000);   // KW_DB 2

    dev_info(dev->ctrl.device, "%s() sq_dma_addr[0x%llX] (Ln %d)\n",
        __func__, (u64) nvmeq->sq_dma_addr, __LINE__); // KW_DB
    dev_info(dev->ctrl.device, "%s() cq_dma_addr[0x%llX] (Ln %d)\n",
        __func__, (u64) nvmeq->cq_dma_addr, __LINE__); // KW_DB

    if (check_addr_align(dev, nvmeq->sq_dma_addr))
        dev_err(dev->ctrl.device, "SQ aligment failed!\n");
    if (check_addr_align(dev, nvmeq->cq_dma_addr))
        dev_err(dev->ctrl.device, "CQ aligment failed!\n");

#if (DYNAMIC_WIDX == (0))
    // Request reverse mapping with LUT.
    pci_write_config_dword(pdev, NTB_NVME_ASQ_ADDR_LO, (u32)(nvmeq->sq_dma_addr & 0xFFFFFFFF));
    pci_write_config_dword(pdev, NTB_NVME_ASQ_ADDR_HI, (u32)(nvmeq->sq_dma_addr >> 32));
    pci_write_config_dword(pdev, NTB_NVME_ACQ_ADDR_LO, (u32)(nvmeq->cq_dma_addr & 0xFFFFFFFF));
    pci_write_config_dword(pdev, NTB_NVME_ACQ_ADDR_HI, (u32)(nvmeq->cq_dma_addr >> 32));
#endif

#if (DYNAMIC_WIDX)
    // qid should be 0 here.
    getFreeWidx(dev, &widx);
    dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n",
        __func__, widx, __LINE__); // KW_DB

    addr32 = (u32)(nvmeq->sq_dma_addr & 0xFFFFFFFF);
    // addr32 |= (widx << ADDR_LO_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_ASQ_ADDR_LO, addr32);

    addr32 = (u32)(nvmeq->sq_dma_addr >> 32);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_ASQ_ADDR_HI, addr32);
#else
    // Use widx 7 for ASQ, Update with remote addresses.
    widx = 7;
#endif

    addr = dev->dnb_addr + ((widx+1) * SZ_32K);
    // lo_hi_writeq(dev->dnb_addr + ((widx+1) * SZ_32K), dev->bar + NVME_REG_ASQ);
    lo_hi_writeq(addr, dev->bar + NVME_REG_ASQ);
    dev_info(dev->ctrl.device, "%s() Wrote addr[0x%llX] to NVME_REG_ASQ (Ln %d)\n",
        __func__, addr, __LINE__); // KW_DB

#if (DYNAMIC_WIDX)
    // qid should be 0 here.
    getFreeWidx(dev, &widx);
    dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n",
        __func__, widx, __LINE__); // KW_DB

    addr32 = (u32)(nvmeq->cq_dma_addr & 0xFFFFFFFF);
    ///addr32 |= (widx << ADDR_LO_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_ACQ_ADDR_LO, addr32);

    addr32 = (u32)(nvmeq->cq_dma_addr >> 32);
    addr32 |= (widx << ADDR_HI_WIDX_SHIFT);
    pci_write_config_dword(pdev, NTB_NVME_ACQ_ADDR_HI, addr32);
#else
    // Use widx 8 for ACQ, Update with remote addresses.
    widx = 8;
#endif

    addr = dev->dnb_addr + ((widx+1) * SZ_32K);
    // lo_hi_writeq(dev->dnb_addr + ((widx+1) * SZ_32K), dev->bar + NVME_REG_ACQ);
    lo_hi_writeq(addr, dev->bar + NVME_REG_ACQ);
    dev_info(dev->ctrl.device, "%s() Wrote addr[0x%llX] to NVME_REG_ACQ (Ln %d)\n",
        __func__, addr, __LINE__); // KW_DB
#else
// KW_DB<-
	lo_hi_writeq(nvmeq->sq_dma_addr, dev->bar + NVME_REG_ASQ);
	lo_hi_writeq(nvmeq->cq_dma_addr, dev->bar + NVME_REG_ACQ);
#endif  // KW_DB

	result = nvme_enable_ctrl(&dev->ctrl);
    
    #if 1   // KW_DB_20240910 - Try not using my workaround  // KW_DB_20250320 - Re-enable for now
    set_msix_enable(dev, __func__, __LINE__); // KW_DB
    #endif  // KW_DB_20240910
    // KW_DB->
    {
        u32 csts = readl(dev->bar + NVME_REG_CSTS);

        dev_info(dev->ctrl.device, "%s() result[%d] NVME_REG_CSTS[0x%X] (Ln %d)\n", __func__, result, csts, __LINE__); // KW_DB
    }
    // KW_DB<-
	if (result)
		return result;

	nvmeq->cq_vector = 0;
	nvme_init_queue(nvmeq, 0);
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	//result = queue_request_irq(nvmeq);           // KW_DB
    result = queue_request_irq(nvmeq, __LINE__);   // KW_DB
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	if (result) {
        dev_info(dev->ctrl.device, "%s() result[%d] (Ln %d)\n", __func__, result, __LINE__); // KW_DB
		dev->online_queues--;
		return result;
	}

	set_bit(NVMEQ_ENABLED, &nvmeq->flags);
	return result;
}

static int nvme_create_io_queues(struct nvme_dev *dev)
{
	unsigned i, max, rw_queues;
	int ret = 0;

	for (i = dev->ctrl.queue_count; i <= dev->max_qid; i++) {
		if (nvme_alloc_queue(dev, i, dev->q_depth)) {
			ret = -ENOMEM;
			break;
		}
	}

	max = min(dev->max_qid, dev->ctrl.queue_count - 1);
	if (max != 1 && dev->io_queues[HCTX_TYPE_POLL]) {
		rw_queues = dev->io_queues[HCTX_TYPE_DEFAULT] +
				dev->io_queues[HCTX_TYPE_READ];
	} else {
		rw_queues = max;
	}

	for (i = dev->online_queues; i <= max; i++) {
		bool polled = i > rw_queues;

		ret = nvme_create_queue(&dev->queues[i], i, polled);
		if (ret)
			break;
	}

	/*
	 * Ignore failing Create SQ/CQ commands, we can continue with less
	 * than the desired amount of queues, and even a controller without
	 * I/O queues can still be used to issue admin commands.  This might
	 * be useful to upgrade a buggy firmware for example.
	 */
	return ret >= 0 ? 0 : ret;
}

static u64 nvme_cmb_size_unit(struct nvme_dev *dev)
{
	u8 szu = (dev->cmbsz >> NVME_CMBSZ_SZU_SHIFT) & NVME_CMBSZ_SZU_MASK;
    dev_info(dev->ctrl.device, "%s() cmbsz[%d] szu[%d] (Ln %d)\n", __func__, dev->cmbsz, szu, __LINE__); // KW_DB

	return 1ULL << (12 + 4 * szu);
}

static u32 nvme_cmb_size(struct nvme_dev *dev)
{
	return (dev->cmbsz >> NVME_CMBSZ_SZ_SHIFT) & NVME_CMBSZ_SZ_MASK;
}

static void nvme_map_cmb(struct nvme_dev *dev)
{
	u64 size, offset;
	resource_size_t bar_size;
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	int bar;

	if (dev->cmb_size)
		return;

	if (NVME_CAP_CMBS(dev->ctrl.cap))
		writel(NVME_CMBMSC_CRE, dev->bar + NVME_REG_CMBMSC);

	dev->cmbsz = readl(dev->bar + NVME_REG_CMBSZ);
	if (!dev->cmbsz)
		return;
	dev->cmbloc = readl(dev->bar + NVME_REG_CMBLOC);

	size = nvme_cmb_size_unit(dev) * nvme_cmb_size(dev);
	offset = nvme_cmb_size_unit(dev) * NVME_CMB_OFST(dev->cmbloc);
	bar = NVME_CMB_BIR(dev->cmbloc);
	bar_size = pci_resource_len(pdev, bar);

	if (offset > bar_size)
		return;

	/*
	 * Tell the controller about the host side address mapping the CMB,
	 * and enable CMB decoding for the NVMe 1.4+ scheme:
	 */
	if (NVME_CAP_CMBS(dev->ctrl.cap)) {
		hi_lo_writeq(NVME_CMBMSC_CRE | NVME_CMBMSC_CMSE |
			     (pci_bus_address(pdev, bar) + offset),
			     dev->bar + NVME_REG_CMBMSC);
	}

	/*
	 * Controllers may support a CMB size larger than their BAR,
	 * for example, due to being behind a bridge. Reduce the CMB to
	 * the reported size of the BAR
	 */
	if (size > bar_size - offset)
		size = bar_size - offset;

	if (pci_p2pdma_add_resource(pdev, bar, size, offset)) {
		dev_warn(dev->ctrl.device,
			 "failed to register the CMB\n");
		return;
	}

	dev->cmb_size = size;
	dev->cmb_use_sqes = use_cmb_sqes && (dev->cmbsz & NVME_CMBSZ_SQS);

	if ((dev->cmbsz & (NVME_CMBSZ_WDS | NVME_CMBSZ_RDS)) ==
			(NVME_CMBSZ_WDS | NVME_CMBSZ_RDS))
		pci_p2pmem_publish(pdev, true);

	nvme_update_attrs(dev);
}

static int nvme_set_host_mem(struct nvme_dev *dev, u32 bits)
{
	u32 host_mem_size = dev->host_mem_size >> NVME_CTRL_PAGE_SHIFT;
	u64 dma_addr = dev->host_mem_descs_dma;
	struct nvme_command c = { };
	int ret;

	c.features.opcode	= nvme_admin_set_features;
	c.features.fid		= cpu_to_le32(NVME_FEAT_HOST_MEM_BUF);
	c.features.dword11	= cpu_to_le32(bits);
	c.features.dword12	= cpu_to_le32(host_mem_size);
	c.features.dword13	= cpu_to_le32(lower_32_bits(dma_addr));
	c.features.dword14	= cpu_to_le32(upper_32_bits(dma_addr));
	c.features.dword15	= cpu_to_le32(dev->nr_host_mem_descs);

	ret = nvme_submit_sync_cmd(dev->ctrl.admin_q, &c, NULL, 0);
	if (ret) {
		dev_warn(dev->ctrl.device,
			 "failed to set host mem (err %d, flags %#x).\n",
			 ret, bits);
	} else
		dev->hmb = bits & NVME_HOST_MEM_ENABLE;

	return ret;
}

static void nvme_free_host_mem(struct nvme_dev *dev)
{
	int i;

	for (i = 0; i < dev->nr_host_mem_descs; i++) {
		struct nvme_host_mem_buf_desc *desc = &dev->host_mem_descs[i];
		size_t size = le32_to_cpu(desc->size) * NVME_CTRL_PAGE_SIZE;

		dma_free_attrs(dev->dev, size, dev->host_mem_desc_bufs[i],
			       le64_to_cpu(desc->addr),
			       DMA_ATTR_NO_KERNEL_MAPPING | DMA_ATTR_NO_WARN);
	}

	kfree(dev->host_mem_desc_bufs);
	dev->host_mem_desc_bufs = NULL;
	dma_free_coherent(dev->dev,
			dev->nr_host_mem_descs * sizeof(*dev->host_mem_descs),
			dev->host_mem_descs, dev->host_mem_descs_dma);
	dev->host_mem_descs = NULL;
	dev->nr_host_mem_descs = 0;
}

static int __nvme_alloc_host_mem(struct nvme_dev *dev, u64 preferred,
		u32 chunk_size)
{
	struct nvme_host_mem_buf_desc *descs;
	u32 max_entries, len;
	dma_addr_t descs_dma;
	int i = 0;
	void **bufs;
	u64 size, tmp;

	tmp = (preferred + chunk_size - 1);
	do_div(tmp, chunk_size);
	max_entries = tmp;

	if (dev->ctrl.hmmaxd && dev->ctrl.hmmaxd < max_entries)
		max_entries = dev->ctrl.hmmaxd;

	descs = dma_alloc_coherent(dev->dev, max_entries * sizeof(*descs),
				   &descs_dma, GFP_KERNEL);
	if (!descs)
		goto out;

	bufs = kcalloc(max_entries, sizeof(*bufs), GFP_KERNEL);
	if (!bufs)
		goto out_free_descs;

	for (size = 0; size < preferred && i < max_entries; size += len) {
		dma_addr_t dma_addr;

		len = min_t(u64, chunk_size, preferred - size);
		bufs[i] = dma_alloc_attrs(dev->dev, len, &dma_addr, GFP_KERNEL,
				DMA_ATTR_NO_KERNEL_MAPPING | DMA_ATTR_NO_WARN);
		if (!bufs[i])
			break;

		descs[i].addr = cpu_to_le64(dma_addr);
		descs[i].size = cpu_to_le32(len / NVME_CTRL_PAGE_SIZE);
		i++;
	}

	if (!size)
		goto out_free_bufs;

	dev->nr_host_mem_descs = i;
	dev->host_mem_size = size;
	dev->host_mem_descs = descs;
	dev->host_mem_descs_dma = descs_dma;
	dev->host_mem_desc_bufs = bufs;
	return 0;

out_free_bufs:
	while (--i >= 0) {
		size_t size = le32_to_cpu(descs[i].size) * NVME_CTRL_PAGE_SIZE;

		dma_free_attrs(dev->dev, size, bufs[i],
			       le64_to_cpu(descs[i].addr),
			       DMA_ATTR_NO_KERNEL_MAPPING | DMA_ATTR_NO_WARN);
	}

	kfree(bufs);
out_free_descs:
	dma_free_coherent(dev->dev, max_entries * sizeof(*descs), descs,
			descs_dma);
out:
	dev->host_mem_descs = NULL;
	return -ENOMEM;
}

static int nvme_alloc_host_mem(struct nvme_dev *dev, u64 min, u64 preferred)
{
	u64 min_chunk = min_t(u64, preferred, PAGE_SIZE * MAX_ORDER_NR_PAGES);
	u64 hmminds = max_t(u32, dev->ctrl.hmminds * 4096, PAGE_SIZE * 2);
	u64 chunk_size;

	/* start big and work our way down */
	for (chunk_size = min_chunk; chunk_size >= hmminds; chunk_size /= 2) {
		if (!__nvme_alloc_host_mem(dev, preferred, chunk_size)) {
			if (!min || dev->host_mem_size >= min)
				return 0;
			nvme_free_host_mem(dev);
		}
	}

	return -ENOMEM;
}

static int nvme_setup_host_mem(struct nvme_dev *dev)
{
	u64 max = (u64)max_host_mem_size_mb * SZ_1M;
	u64 preferred = (u64)dev->ctrl.hmpre * 4096;
	u64 min = (u64)dev->ctrl.hmmin * 4096;
	u32 enable_bits = NVME_HOST_MEM_ENABLE;
	int ret;

	if (!dev->ctrl.hmpre)
		return 0;

	preferred = min(preferred, max);
	if (min > max) {
		dev_warn(dev->ctrl.device,
			"min host memory (%lld MiB) above limit (%d MiB).\n",
			min >> ilog2(SZ_1M), max_host_mem_size_mb);
		nvme_free_host_mem(dev);
		return 0;
	}

	/*
	 * If we already have a buffer allocated check if we can reuse it.
	 */
	if (dev->host_mem_descs) {
		if (dev->host_mem_size >= min)
			enable_bits |= NVME_HOST_MEM_RETURN;
		else
			nvme_free_host_mem(dev);
	}

	if (!dev->host_mem_descs) {
		if (nvme_alloc_host_mem(dev, min, preferred)) {
			dev_warn(dev->ctrl.device,
				"failed to allocate host memory buffer.\n");
			return 0; /* controller must work without HMB */
		}

		dev_info(dev->ctrl.device,
			"allocated %lld MiB host memory buffer.\n",
			dev->host_mem_size >> ilog2(SZ_1M));
	}

	ret = nvme_set_host_mem(dev, enable_bits);
	if (ret)
		nvme_free_host_mem(dev);
	return ret;
}

static ssize_t cmb_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct nvme_dev *ndev = to_nvme_dev(dev_get_drvdata(dev));

	return sysfs_emit(buf, "cmbloc : x%08x\ncmbsz  : x%08x\n",
		       ndev->cmbloc, ndev->cmbsz);
}
static DEVICE_ATTR_RO(cmb);

static ssize_t cmbloc_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct nvme_dev *ndev = to_nvme_dev(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%u\n", ndev->cmbloc);
}
static DEVICE_ATTR_RO(cmbloc);

static ssize_t cmbsz_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct nvme_dev *ndev = to_nvme_dev(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%u\n", ndev->cmbsz);
}
static DEVICE_ATTR_RO(cmbsz);

static ssize_t hmb_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct nvme_dev *ndev = to_nvme_dev(dev_get_drvdata(dev));

	return sysfs_emit(buf, "%d\n", ndev->hmb);
}

static ssize_t hmb_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct nvme_dev *ndev = to_nvme_dev(dev_get_drvdata(dev));
	bool new;
	int ret;

	if (kstrtobool(buf, &new) < 0)
		return -EINVAL;

	if (new == ndev->hmb)
		return count;

	if (new) {
		ret = nvme_setup_host_mem(ndev);
	} else {
		ret = nvme_set_host_mem(ndev, 0);
		if (!ret)
			nvme_free_host_mem(ndev);
	}

	if (ret < 0)
		return ret;

	return count;
}
static DEVICE_ATTR_RW(hmb);

static umode_t nvme_pci_attrs_are_visible(struct kobject *kobj,
		struct attribute *a, int n)
{
	struct nvme_ctrl *ctrl =
		dev_get_drvdata(container_of(kobj, struct device, kobj));
	struct nvme_dev *dev = to_nvme_dev(ctrl);

	if (a == &dev_attr_cmb.attr ||
	    a == &dev_attr_cmbloc.attr ||
	    a == &dev_attr_cmbsz.attr) {
	    	if (!dev->cmbsz)
			return 0;
	}
	if (a == &dev_attr_hmb.attr && !ctrl->hmpre)
		return 0;

	return a->mode;
}

static struct attribute *nvme_pci_attrs[] = {
	&dev_attr_cmb.attr,
	&dev_attr_cmbloc.attr,
	&dev_attr_cmbsz.attr,
	&dev_attr_hmb.attr,
	NULL,
};

static const struct attribute_group nvme_pci_dev_attrs_group = {
	.attrs		= nvme_pci_attrs,
	.is_visible	= nvme_pci_attrs_are_visible,
};

static const struct attribute_group *nvme_pci_dev_attr_groups[] = {
	&nvme_dev_attrs_group,
	&nvme_pci_dev_attrs_group,
	NULL,
};

static void nvme_update_attrs(struct nvme_dev *dev)
{
	sysfs_update_group(&dev->ctrl.device->kobj, &nvme_pci_dev_attrs_group);
}

#if 0 // KW_DB_20240813
/*
 * nirqs is the number of interrupts available for write and read
 * queues. The core already reserved an interrupt for the admin queue.
 */
static void nvme_calc_irq_sets(struct irq_affinity *affd, unsigned int nrirqs)
{
	struct nvme_dev *dev = affd->priv;
	unsigned int nr_read_queues, nr_write_queues = dev->nr_write_queues;

	/*
	 * If there is no interrupt available for queues, ensure that
	 * the default queue is set to 1. The affinity set size is
	 * also set to one, but the irq core ignores it for this case.
	 *
	 * If only one interrupt is available or 'write_queue' == 0, combine
	 * write and read queues.
	 *
	 * If 'write_queues' > 0, ensure it leaves room for at least one read
	 * queue.
	 */
	if (!nrirqs) {
		nrirqs = 1;
		nr_read_queues = 0;
	} else if (nrirqs == 1 || !nr_write_queues) {
		nr_read_queues = 0;
	} else if (nr_write_queues >= nrirqs) {
		nr_read_queues = 1;
	} else {
		nr_read_queues = nrirqs - nr_write_queues;
	}

	dev->io_queues[HCTX_TYPE_DEFAULT] = nrirqs - nr_read_queues;
	affd->set_size[HCTX_TYPE_DEFAULT] = nrirqs - nr_read_queues;
	dev->io_queues[HCTX_TYPE_READ] = nr_read_queues;
	affd->set_size[HCTX_TYPE_READ] = nr_read_queues;
	affd->nr_sets = nr_read_queues ? 2 : 1;
}
#endif  // KW_DB_20240813

static int nvme_setup_irqs(struct nvme_dev *dev, unsigned int nr_io_queues)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);
////*  // KW_DB_20241112 - Trying to revert changes of KW_DB_20240813
    #if 0 // KW_DB_20240813
///*/  // KW_DB_20241112
	struct irq_affinity affd = {
		.pre_vectors	= 1,
		.calc_sets	= nvme_calc_irq_sets,
		.priv		= dev,
	};
////*  // KW_DB_20241112 - Trying to revert changes of KW_DB_20240813
    #endif
///*/  // KW_DB_20241112
	unsigned int irq_queues, poll_queues;

	/*
	 * Poll queues don't need interrupts, but we need at least one I/O queue
	 * left over for non-polled I/O.
	 */
	poll_queues = min(dev->nr_poll_queues, nr_io_queues - 1);
    pr_info("%s() poll_queues[%d] (Ln %d)\n", __func__, poll_queues, __LINE__); // KW_DB_20241112
	dev->io_queues[HCTX_TYPE_POLL] = poll_queues;

	/*
	 * Initialize for the single interrupt case, will be updated in
	 * nvme_calc_irq_sets().
	 */
	dev->io_queues[HCTX_TYPE_DEFAULT] = 1;
	dev->io_queues[HCTX_TYPE_READ] = 0;

	/*
	 * We need interrupts for the admin queue and each non-polled I/O queue,
	 * but some Apple controllers require all queues to use the first
	 * vector.
	 */
	irq_queues = 1;
	if (!(dev->ctrl.quirks & NVME_QUIRK_SINGLE_VECTOR))
		irq_queues += (nr_io_queues - poll_queues);
    dev_info(dev->ctrl.device, "%s(): Calling pci_alloc_irq_vectors_affinity() with irq_queues[%d] cpu[%d] (Ln %d)\n",
        __func__, irq_queues, current->on_cpu, __LINE__); // KW_DB
    dev->nr_irq_queues = irq_queues;    // KW_DB_20250116
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
    
    #if 1 // KW_DB_20240813
    return pci_alloc_irq_vectors(pdev, 1, irq_queues, PCI_IRQ_ALL_TYPES);
    #else
	return pci_alloc_irq_vectors_affinity(pdev, 1, irq_queues,
			      // PCI_IRQ_ALL_TYPES | PCI_IRQ_AFFINITY, &affd);  // KW_DB
			      PCI_IRQ_ALL_TYPES, &affd);    // KW_DB
    #endif
}

static unsigned int nvme_max_io_queues(struct nvme_dev *dev)
{
	/*
	 * If tags are shared with admin queue (Apple bug), then
	 * make sure we only use one IO queue.
	 */
	if (dev->ctrl.quirks & NVME_QUIRK_SHARED_TAGS)
		return 1;
	// return num_possible_cpus() + dev->nr_write_queues + dev->nr_poll_queues; // KW_DB_20241112
    return (1 + dev->nr_write_queues + dev->nr_poll_queues);                    // KW_DB_20241112 - Should be 1 + 0 + 4 now.
}

static int nvme_setup_io_queues(struct nvme_dev *dev)
{
	struct nvme_queue *adminq = &dev->queues[0];
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	unsigned int nr_io_queues;
	unsigned long size;
	int result;

	/*
	 * Sample the module parameters once at reset time so that we have
	 * stable values to work with.
	 */
	dev->nr_write_queues = write_queues;
	dev->nr_poll_queues = poll_queues;

    pr_info("%s() nr_allocated_queues[%d] (Ln %d)\n", __func__, dev->nr_allocated_queues, __LINE__);    // KW_DB_20241112
    pr_info("%s() nr_poll_queues[%d] (Ln %d)\n", __func__, dev->nr_poll_queues, __LINE__);              // KW_DB_20241112
    
	nr_io_queues = dev->nr_allocated_queues - 1;
	result = nvme_set_queue_count(&dev->ctrl, &nr_io_queues);
	if (result < 0)
		return result;

    pr_info("%s() nr_io_queues[%d] (Ln %d)\n", __func__, nr_io_queues, __LINE__);   // KW_DB_20250116 - debug polling queues
	if (nr_io_queues == 0)
		return 0;

    pr_info("%s() Here. (Ln %d)\n", __func__, __LINE__);   // KW_DB_20250116 - debug polling queues
	/*
	 * Free IRQ resources as soon as NVMEQ_ENABLED bit transitions
	 * from set to unset. If there is a window to it is truely freed,
	 * pci_free_irq_vectors() jumping into this window will crash.
	 * And take lock to avoid racing with pci_free_irq_vectors() in
	 * nvme_dev_disable() path.
	 */
	result = nvme_setup_io_queues_trylock(dev);
	if (result)
		return result;
    pr_info("%s() Here. (Ln %d)\n", __func__, __LINE__);   // KW_DB_20250116 - debug polling queues
	if (test_and_clear_bit(NVMEQ_ENABLED, &adminq->flags))
    {
        dev_info(dev->ctrl.device, "%s() Calling pci_free_irq(0) (Ln %d)\n", __func__, __LINE__); // KW_DB
		pci_free_irq(pdev, 0, adminq);
        clear_msix_alloc_tracker(dev, 0);   // KW_DB_20240828
    }

	if (dev->cmb_use_sqes) {
		result = nvme_cmb_qdepth(dev, nr_io_queues,
				sizeof(struct nvme_command));
		if (result > 0) {
			dev->q_depth = result;
			dev->ctrl.sqsize = result - 1;
		} else {
			dev->cmb_use_sqes = false;
		}
	}

	do {
		size = db_bar_size(dev, nr_io_queues);
        dev_info(dev->ctrl.device, "%s() Calling nvme_remap_bar() with size[0x%lX] (Ln %d)\n",
            __func__, (unsigned long) size, __LINE__); // KW_DB
		result = nvme_remap_bar(dev, size);
		if (!result)
			break;
		if (!--nr_io_queues) {
			result = -ENOMEM;
			goto out_unlock;
		}
	} while (1);
	adminq->q_db = dev->dbs;

 retry:
	/* Deregister the admin queue's interrupt */
	if (test_and_clear_bit(NVMEQ_ENABLED, &adminq->flags))
    {
        dev_info(dev->ctrl.device, "%s() Calling pci_free_irq(0) (Ln %d)\n", __func__, __LINE__); // KW_DB
		pci_free_irq(pdev, 0, adminq);
        clear_msix_alloc_tracker(dev, 0);   // KW_DB_20240828
    }

	/*
	 * If we enable msix early due to not intx, disable it again before
	 * setting up the full range we need.
	 */
	pci_free_irq_vectors(pdev);

    // TODO: address below - KW_DB
	result = nvme_setup_irqs(dev, nr_io_queues);
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	if (result <= 0) {
		result = -EIO;
		goto out_unlock;
	}

// #ifdef ULTRA
    // dev_info(dev->ctrl.device, "%s(): nr_allocated_queues[%d] (Ln %d)\n",
        // __func__, dev->nr_allocated_queues, __LINE__); // KW_DB
    // dump_msix_vectors(dev, dev->nr_allocated_queues);
    // rewrite_msix_table(dev, dev->nr_allocated_queues);
// #endif  /* ULTRA */

	dev->num_vecs = result;
	result = max(result - 1, 1);
	dev->max_qid = result + dev->io_queues[HCTX_TYPE_POLL];

    dev_info(dev->ctrl.device, "%s(): Here (Ln %d)\n",
        __func__, __LINE__); // KW_DB
    ///clear_msix_table_entries(dev,  // KW_DB

	/*
	 * Should investigate if there's a performance win from allocating
	 * more queues than interrupt vectors; it might allow the submission
	 * path to scale better, even if the receive path is limited by the
	 * number of interrupts.
	 */
    // TODO: address below (referce line #3338, etc - KW_DB
	//result = queue_request_irq(adminq);           // KW_DB
    result = queue_request_irq(adminq, __LINE__);   // KW_DB
	if (result)
		goto out_unlock;
	set_bit(NVMEQ_ENABLED, &adminq->flags);
	mutex_unlock(&dev->shutdown_lock);

#ifdef ULTRA

    ///clear_msix_table_entries(dev, dev->nr_allocated_queues +1); // KW_DB

    dump_msix_table_entry(dev, 0, 4, __LINE__);
    rewrite_msix_table_entry(dev, 0);

    // mdelay(2000);   // KW_DB
    // dev_info(dev->ctrl.device, "%s(): nr_allocated_queues[%d] (Ln %d)\n",
        // __func__, dev->nr_allocated_queues, __LINE__); // KW_DB
    // dump_msix_vectors(dev, dev->nr_allocated_queues);
    // rewrite_msix_table(dev, dev->nr_allocated_queues);
#endif  /* ULTRA */

    dev_info(dev->ctrl.device, "%s(): Here (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	result = nvme_create_io_queues(dev);
    dev_info(dev->ctrl.device, "%s(): result[%d] (Ln %d)\n",
        __func__, result, __LINE__); // KW_DB

	if (result || dev->online_queues < 2)
    {   // KW_DB
        dev_err(dev->ctrl.device, "%s(): nresult[%d] online_queues[%d] (Ln %d)\n",
            __func__, result, dev->online_queues, __LINE__); // KW_DB
		return result;
    }   // KW_DB

    dev_info(dev->ctrl.device, "%s(): Here (Ln %d)\n",
        __func__, __LINE__); // KW_DB

// #ifdef ULTRA
    // mdelay(2000);   // KW_DB
    // dev_info(dev->ctrl.device, "%s(): nr_allocated_queues[%d] (Ln %d)\n",
        // __func__, dev->nr_allocated_queues, __LINE__); // KW_DB
    // dump_msix_vectors(dev, dev->nr_allocated_queues);
    // rewrite_msix_table(dev, dev->nr_allocated_queues);
// #endif  /* ULTRA */

	if (dev->online_queues - 1 < dev->max_qid) {
		nr_io_queues = dev->online_queues - 1;
		nvme_delete_io_queues(dev);
		result = nvme_setup_io_queues_trylock(dev);
		if (result)
			return result;
		nvme_suspend_io_queues(dev);
		goto retry;
	}
	dev_info(dev->ctrl.device, "%d/%d/%d default/read/poll queues\n",
					dev->io_queues[HCTX_TYPE_DEFAULT],
					dev->io_queues[HCTX_TYPE_READ],
					dev->io_queues[HCTX_TYPE_POLL]);
	return 0;
out_unlock:
	mutex_unlock(&dev->shutdown_lock);
	return result;
}

static enum rq_end_io_ret nvme_del_queue_end(struct request *req,
					     blk_status_t error)
{
	struct nvme_queue *nvmeq = req->end_io_data;

	blk_mq_free_request(req);
	complete(&nvmeq->delete_done);
	return RQ_END_IO_NONE;
}

static enum rq_end_io_ret nvme_del_cq_end(struct request *req,
					  blk_status_t error)
{
	struct nvme_queue *nvmeq = req->end_io_data;

	if (error)
		set_bit(NVMEQ_DELETE_ERROR, &nvmeq->flags);

	return nvme_del_queue_end(req, error);
}

static int nvme_delete_queue(struct nvme_queue *nvmeq, u8 opcode)
{
	struct request_queue *q = nvmeq->dev->ctrl.admin_q;
	struct request *req;
	struct nvme_command cmd = { };

	cmd.delete_queue.opcode = opcode;
	cmd.delete_queue.qid = cpu_to_le16(nvmeq->qid);

	req = blk_mq_alloc_request(q, nvme_req_op(&cmd), BLK_MQ_REQ_NOWAIT);
	if (IS_ERR(req))
		return PTR_ERR(req);
	nvme_init_request(req, &cmd);

	if (opcode == nvme_admin_delete_cq)
		req->end_io = nvme_del_cq_end;
	else
		req->end_io = nvme_del_queue_end;
	req->end_io_data = nvmeq;

	init_completion(&nvmeq->delete_done);
	blk_execute_rq_nowait(req, false);
	return 0;
}

static bool __nvme_delete_io_queues(struct nvme_dev *dev, u8 opcode)
{
	int nr_queues = dev->online_queues - 1, sent = 0;
	unsigned long timeout;

 retry:
	timeout = NVME_ADMIN_TIMEOUT;
	while (nr_queues > 0) {
		if (nvme_delete_queue(&dev->queues[nr_queues], opcode))
			break;
		nr_queues--;
		sent++;
	}
	while (sent) {
		struct nvme_queue *nvmeq = &dev->queues[nr_queues + sent];

		timeout = wait_for_completion_io_timeout(&nvmeq->delete_done,
				timeout);
		if (timeout == 0)
			return false;

		sent--;
		if (nr_queues)
			goto retry;
	}
	return true;
}

static void nvme_delete_io_queues(struct nvme_dev *dev)
{
	if (__nvme_delete_io_queues(dev, nvme_admin_delete_sq))
		__nvme_delete_io_queues(dev, nvme_admin_delete_cq);
}

static unsigned int nvme_pci_nr_maps(struct nvme_dev *dev)
{
	if (dev->io_queues[HCTX_TYPE_POLL])
		return 3;
	if (dev->io_queues[HCTX_TYPE_READ])
		return 2;
	return 1;
}

static void nvme_pci_update_nr_queues(struct nvme_dev *dev)
{
	blk_mq_update_nr_hw_queues(&dev->tagset, dev->online_queues - 1);
	/* free previously allocated queues that are no longer usable */
	nvme_free_queues(dev, dev->online_queues);
}

// KW_DB->
#if 0   // Not useful
static int nvme_ntb_probe(struct ntb_client *client, struct ntb_dev *ntb)
{
    pr_info("%s() Entered. (Ln %d)\n", __func__, __LINE__);
    if (ntb_temp_holder)
    {
        pr_err("%s() ntb_temp_holder should be NULL (Ln %d)\n", __func__, __LINE__);
    }
    pr_info("%s() ntb->ctx(%p) (Ln %d)\n", __func__, ntb->ctx, __LINE__);
    ntb_temp_holder = ntb;
	return 0;
}

static void nvme_ntb_remove(struct ntb_client *client, struct ntb_dev *ntb)
{
    pr_info("%s() Entered. (Ln %d)\n", __func__, __LINE__);
}

static struct ntb_client nvme_ntb_client = {
	.ops = {
		.probe = nvme_ntb_probe,
		.remove = nvme_ntb_remove
	}
};
#endif

#if 0   // Moved up



/**
 *
 */
static void read_msix_vector(struct nvme_dev *dev, int idx, u32 *dword)
{
    struct pci_dev *pdev = to_pci_dev(dev->dev);
    void __iomem   *msix_table = NULL;
    u8              msix_pos;
    u8              msix_bar;
    u32             msix_table_offset;
    u16             msix_table_size;

    msix_pos = pdev->msix_cap;
    if (msix_pos)
    {
        u16 flags;
        u32 table;

        pci_read_config_word(pdev, msix_pos + PCI_MSIX_FLAGS, &flags);
        pci_read_config_dword(pdev, msix_pos + PCI_MSIX_TABLE, &table);

        msix_bar = table & PCI_MSIX_TABLE_BIR;
        msix_table_offset = table & PCI_MSIX_TABLE_OFFSET;
        msix_table_size = ((flags & PCI_MSIX_FLAGS_QSIZE) + 1) * 16;

        dev_err(dev->ctrl.device, "%s() msix_table_offset[0x%X]  (Ln %d)\n", __func__, msix_table_offset, __LINE__); // KW_DB

        msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
        if (msix_table)
        {
            // u32 dword = 0;
            int i;

            for (i=0; i<4; i++)
            {
                dword[i] = readl(msix_table+(i*4) + (4*idx));
                // dev_info(dev->ctrl.device, "%s() irqVector+%04d[0x%X] (Ln %d)\n",
                    // __func__, (i*4)+(4*idx), dword[i], __LINE__);
            }

            iounmap(msix_table);
        }
        else
        {
            dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
        }
    }
}
#endif

// KW_DB<-

static int nvme_pci_enable(struct nvme_dev *dev)
{
    // KW_DB->
    void __iomem   *msix_table = NULL;
    u8              msix_pos;
    u8              msix_bar;
    u32             msix_table_offset;
    u16             msix_table_size;
    u32 csts;
    // KW_DB<-

	int result = -ENOMEM;
	struct pci_dev *pdev = to_pci_dev(dev->dev);
    
    csts = readl(dev->bar + NVME_REG_CSTS); // KW_DB
    dev_info(dev->ctrl.device, "%s() NVME_REG_CSTS[0x%X] (Ln %d)\n", __func__, csts, __LINE__); // KW_DB

	if (pci_enable_device_mem(pdev))
		return result;

	pci_set_master(pdev);

    csts = readl(dev->bar + NVME_REG_CSTS); // KW_DB
    dev_info(dev->ctrl.device, "%s() NVME_REG_CSTS[0x%X] (Ln %d)\n", __func__, csts, __LINE__); // KW_DB


	if (readl(dev->bar + NVME_REG_CSTS) == -1) {
		result = -ENODEV;
		goto disable;
	}

    // KW_DB->

   // https://elixir.bootlin.com/linux/v6.2.16/source/drivers/vfio/pci/vfio_pci_core.c#L530


    msix_pos = pdev->msix_cap;

    // Retrieve MSI-X parameters.
    if (msix_pos)
    {
        u16 flags;
        u32 table;
        u32 val;




        pci_read_config_word(pdev, msix_pos + PCI_MSIX_FLAGS, &flags);
        pci_read_config_dword(pdev, msix_pos + PCI_MSIX_TABLE, &table);

        msix_bar = table & PCI_MSIX_TABLE_BIR;
        msix_table_offset = table & PCI_MSIX_TABLE_OFFSET;
        msix_table_size = ((flags & PCI_MSIX_FLAGS_QSIZE) + 1) * 16;

        dev_warn(dev->ctrl.device, "%s() msix_bar[%d] (Ln %d)\n", __func__, msix_bar, __LINE__);
        dev_warn(dev->ctrl.device, "%s() msix_table_offset[0x%X]  (Ln %d)\n", __func__, msix_table_offset, __LINE__);
        dev_warn(dev->ctrl.device, "%s() msix_table_size[0x%X] (Ln %d)\n", __func__, msix_table_size, __LINE__);
        // Let the NTB driver know.
        pci_write_config_dword(pdev, NTB_NVME_MSIX_TBL_OFFSET, msix_table_offset);
        pci_write_config_dword(pdev, NTB_NVME_MSIX_TBL_SIZE, msix_table_size);
        // Get the NTB BAR1 of lender.
        pci_read_config_dword(pdev, NTB_NVME_PEER_DNB_LO, &val);
        dev->dnb_addr_lo = val;
        pci_read_config_dword(pdev, NTB_NVME_PEER_DNB_HI, &val);
        dev->dnb_addr_hi = val;
        dev->dnb_addr = (dma_addr_t) dev->dnb_addr_hi;
        dev->dnb_addr <<= 32;
        dev->dnb_addr |= (dma_addr_t) dev->dnb_addr_lo ;
        // KW_DB_20241211->
        // Get the NTB 1st MW address of lender.
        pci_read_config_dword(pdev, NTB_NVME_PEER_MW0_LO, &val);
        dev->mw0_addr_lo = val;
        pci_read_config_dword(pdev, NTB_NVME_PEER_MW0_HI, &val);
        dev->mw0_addr_hi = val;
        dev->mw0_addr = (dma_addr_t) dev->mw0_addr_hi;
        dev->mw0_addr <<= 32;
        dev->mw0_addr |= (dma_addr_t) dev->mw0_addr_lo ;
        // KW_DB_20241211<-
        dev_warn(dev->ctrl.device, "%s() dnb_addr_hi[0x%X] dnb_addr_lo[0x%X] (Ln %d)\n", __func__, dev->dnb_addr_hi, dev->dnb_addr_lo, __LINE__);
        dev_warn(dev->ctrl.device, "%s() dnb_addr[0x%llX] (Ln %d)\n", __func__, dev->dnb_addr, __LINE__);
        dev_warn(dev->ctrl.device, "%s() mw0_addr[0x%llX] (Ln %d)\n", __func__, dev->mw0_addr, __LINE__);   // KW_DB_20241211
        // TODO: Obsolete dev->dnb_addr_lo and dev->dnb_addr_hi when ready.

    }


    if (!ntb_temp_holder)
    {
        dev_err(dev->ctrl.device, "%s() ntb_temp_holder should not be NULL (Ln %d)\n", __func__, __LINE__);

    }


    // result = ntb_register_client(&nvme_ntb_client);
    // if (result)
        // dev_err(dev->ctrl.device, "%s() ntb_register_client() result[%d] (Ln %d)\n", __func__, result,__LINE__);

    #if 0  // Not useful
    dev->ntb = ntb_temp_holder;

    if (dev->ntb)
    {
        dev_info(dev->ctrl.device, "%s() dev->ntb(%p) (Ln %d)\n", __func__, dev->ntb, __LINE__);

    }
    else
    {
        dev_err(dev->ctrl.device, "%s() dev->ntb is NULL (Ln %d)\n", __func__, __LINE__);
    }
    #endif

    msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
    if (msix_table)
    {
        u32 irqVector = 0;
        u32 tmp = 0;

        irqVector = readl(msix_table);
        dev_info(dev->ctrl.device, "%s() irqVector+[0x%X] (Ln %d)\n", __func__, irqVector, __LINE__); // KW_DB
        tmp = readl(msix_table+4);
        dev_info(dev->ctrl.device, "%s() irqVector+04[0x%X] (Ln %d)\n", __func__, tmp, __LINE__); // KW_DB
        tmp = readl(msix_table+8);
        dev_info(dev->ctrl.device, "%s() irqVector+08[0x%X] (Ln %d)\n", __func__, tmp, __LINE__); // KW_DB
        tmp = readl(msix_table+12);
        dev_info(dev->ctrl.device, "%s() irqVector+12[0x%X] (Ln %d)\n", __func__, tmp, __LINE__); // KW_DB

        iounmap(msix_table);
    }
    else
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
    // KW_DB<-

	/*
	 * Some devices and/or platforms don't advertise or work with INTx
	 * interrupts. Pre-enable a single MSIX or MSI vec for setup. We'll
	 * adjust this later.
	 */
    dev_info(dev->ctrl.device, "%s() Calling pci_alloc_irq_vectors() cpu[%d] (Ln %d)\n",
        __func__, current->on_cpu, __LINE__); // KW_DB
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	result = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
    dev_info(dev->ctrl.device, "%s() result[%d] (Ln %d)\n",
        __func__, result, __LINE__); // KW_DB
    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	if (result < 0)
		goto disable;

    // KW_DB->
    #if 1
    {
        // Use widx 6 for MSI-X. Update MSI-X table entry with remote addresses.
        int widx = 6;

        /* Get the LUT alignment info. */
        // pci_write_config_dword(pdev, NTB_NVME_ALIGN_SET_WIDX, (u32) widx);
        // pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_XLAT, &dev->lut_xlat_align);
        // pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE, &dev->lut_size_align);
        // pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE_MAX, &dev->lut_size_max);
        // dev_info(dev->ctrl.device, "%s() lut_xlat_align[0x%X] lut_size_align[0x%X] lut_size_max[0x%X] (Ln %d)\n",
            // __func__, dev->lut_xlat_align, dev->lut_size_align, dev->lut_size_max, __LINE__); // KW_DB
        update_align_info(dev, widx);

        // mdelay(5000);   // KW_DB

    }
    #else   // Moved the following to lower...
    msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
    if (msix_table)
    {
        u32 widx;
        u32 irqVector = 0;
        u32 tmp = 0;

        #if 1
        getFreeWidx(dev, &widx);
        dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n",
            __func__, widx, __LINE__); // KW_DB
        #else
        // Use widx 6 for MSI-X. Update MSI-X table entry with remote addresses.
        widx = 6;
        #endif

        /* Get the LUT alignment info. */
        pci_write_config_dword(pdev, NTB_NVME_ALIGN_SET_WIDX, (u32) widx);
        pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_XLAT, &dev->lut_xlat_align);
        pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE, &dev->lut_size_align);
        pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE_MAX, &dev->lut_size_max);
        dev_info(dev->ctrl.device, "%s() lut_xlat_align[0x%X] lut_size_align[0x%X] lut_size_max[0x%X] (Ln %d)\n",
            __func__, dev->lut_xlat_align, dev->lut_size_align, dev->lut_size_max, __LINE__); // KW_DB

        // Read Kernel programmed values.
        irqVector = readl(msix_table);
        dev_info(dev->ctrl.device, "%s() irqVector+00[0x%X] (Ln %d)\n", __func__, irqVector, __LINE__); // KW_DB
        tmp = readl(msix_table+4);
        dev_info(dev->ctrl.device, "%s() irqVector+04[0x%X] (Ln %d)\n", __func__, tmp, __LINE__); // KW_DB
        tmp = readl(msix_table+8);
        dev_info(dev->ctrl.device, "%s() irqVector+08[0x%X] (Ln %d)\n", __func__, tmp, __LINE__); // KW_DB
        tmp = readl(msix_table+12);
        dev_info(dev->ctrl.device, "%s() irqVector+12[0x%X] (Ln %d)\n", __func__, tmp, __LINE__); // KW_DB

        if (check_addr_align(dev, irqVector))
            dev_err(dev->ctrl.device, "IRQ vector aligment failed!\n");

        // Request reverse mapping with LUT.
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO, irqVector);
        irqVector = readl(msix_table+4);
        dev_info(dev->ctrl.device, "%s() irqVector+04[0x%X] (Ln %d)\n", __func__, irqVector, __LINE__); // KW_DB
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI, irqVector);



        // writel(dev->dnb_addr_lo + (6 * SZ_32K), msix_table);
        // writel(dev->dnb_addr_hi, msix_table+4);
        lo_hi_writeq(dev->dnb_addr_lo + ((widx+1) * SZ_32K), msix_table);

        // Enable the following for checking updated MSI-X table entry.
        /*
        irqVector = readl(msix_table);
        dev_info(dev->ctrl.device, "%s() Updated irqVector+00[0x%X] (Ln %d)\n", __func__, irqVector, __LINE__); // KW_DB
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO, irqVector);
        irqVector = readl(msix_table+4);
        dev_info(dev->ctrl.device, "%s() UPdated irqVector+04[0x%X] (Ln %d)\n", __func__, irqVector, __LINE__); // KW_DB
        */
        iounmap(msix_table);


    }
    else
    {
        dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
    }
    #endif
    // KW_DB<-

	dev->ctrl.cap = lo_hi_readq(dev->bar + NVME_REG_CAP);
    dev_info(dev->ctrl.device, "%s() NVME_REG_CAP[0x%llX] (Ln %d)\n", __func__, dev->ctrl.cap, __LINE__); // KW_DB

	dev->q_depth = min_t(u32, NVME_CAP_MQES(dev->ctrl.cap) + 1,
				io_queue_depth);
	dev->db_stride = 1 << NVME_CAP_STRIDE(dev->ctrl.cap);
	dev->dbs = dev->bar + 4096;

	/*
	 * Some Apple controllers require a non-standard SQE size.
	 * Interestingly they also seem to ignore the CC:IOSQES register
	 * so we don't bother updating it here.
	 */
	if (dev->ctrl.quirks & NVME_QUIRK_128_BYTES_SQES)
		dev->io_sqes = 7;
	else
		dev->io_sqes = NVME_NVM_IOSQES;

	/*
	 * Temporary fix for the Apple controller found in the MacBook8,1 and
	 * some MacBook7,1 to avoid controller resets and data loss.
	 */
	if (pdev->vendor == PCI_VENDOR_ID_APPLE && pdev->device == 0x2001) {
		dev->q_depth = 2;
		dev_warn(dev->ctrl.device, "detected Apple NVMe controller, "
			"set queue depth=%u to work around controller resets\n",
			dev->q_depth);
	} else if (pdev->vendor == PCI_VENDOR_ID_SAMSUNG &&
		   (pdev->device == 0xa821 || pdev->device == 0xa822) &&
		   NVME_CAP_MQES(dev->ctrl.cap) == 0) {
		dev->q_depth = 64;
		dev_err(dev->ctrl.device, "detected PM1725 NVMe controller, "
                        "set queue depth=%u\n", dev->q_depth);
	}

	/*
	 * Controllers with the shared tags quirk need the IO queue to be
	 * big enough so that we get 32 tags for the admin queue
	 */
	if ((dev->ctrl.quirks & NVME_QUIRK_SHARED_TAGS) &&
	    (dev->q_depth < (NVME_AQ_DEPTH + 2))) {
		dev->q_depth = NVME_AQ_DEPTH + 2;
		dev_warn(dev->ctrl.device, "IO queue depth clamped to %d\n",
			 dev->q_depth);
	}
	dev->ctrl.sqsize = dev->q_depth - 1; /* 0's based queue depth */

    read_msix_cap(dev, __func__, __LINE__); // KW_DB
	nvme_map_cmb(dev);

	pci_save_state(pdev);

	result = nvme_pci_configure_admin_queue(dev);
	if (result)
    {
        dev_info(dev->ctrl.device, "%s(): result[%d] (Ln %d)\n", __func__, result, __LINE__);
		goto free_irq;
    }

    // KW_DB->
#ifdef ULTRA
    dump_msix_vectors(dev, 1);
    // rewrite_msix_table(dev, 1);      // KW_DB_20240824
    rewrite_msix_table_entry(dev, 0);   // KW_DB_20240824
#else
    {
        u32 dword[4];
        u32 realign_addr_lo;
        u32 offset;
#if (DYNAMIC_WIDX)
        u32 addr32;
#endif
        ///u32 tmp;
        u32 widx;
        int i;

        dev_info(dev->ctrl.device, "%s():Calling read_msix_vector() (Ln %d)\n", __func__, __LINE__);
        read_msix_vector(dev, 0, dword);
        for (i=0; i<4; i++)
        {
            dev_info(dev->ctrl.device, "%s() msi-x dword+%02d[0x%X] (Ln %d)\n", __func__, i, dword[i], __LINE__);
        }

#if (DYNAMIC_WIDX)
        getFreeWidx(dev, &widx);
        dev_info(dev->ctrl.device, "%s() widx[0x%X] allocated. (Ln %d)\n",
            __func__, widx, __LINE__); // KW_DB
#else
        // Use widx 6 for MSI-X. Update MSI-X table entry with remote addresses.
        widx = 6;
#endif

        read_msix_cap(dev, __func__, __LINE__); // KW_DB
        #if 0 // Below should have already performed by now
        /* Get the LUT alignment info. */
        pci_write_config_dword(pdev, NTB_NVME_ALIGN_SET_WIDX, (u32) widx);
        pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_XLAT, &dev->lut_xlat_align);
        pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE, &dev->lut_size_align);
        pci_read_config_dword(pdev, NTB_NVME_ALIGN_GET_SIZE_MAX, &dev->lut_size_max);
        dev_info(dev->ctrl.device, "%s() lut_xlat_align[0x%X] lut_size_align[0x%X] lut_size_max[0x%X] (Ln %d)\n",
            __func__, dev->lut_xlat_align, dev->lut_size_align, dev->lut_size_max, __LINE__); // KW_DB
        #endif

        // mdelay(5000);   // KW_DB

        ///tmp = round_down(dword[0], dev->lut_xlat_align);
        // Request reverse mapping with LUT.
        realign_addr_lo = my_round_down(dev, dword[0]);
        offset = dword[0] - realign_addr_lo;
#if (DYNAMIC_WIDX)
    // qid should be 0 here.
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO, realign_addr_lo);
        addr32 = dword[1] | (widx << ADDR_HI_WIDX_SHIFT);
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI, addr32);
#else
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_LO, realign_addr_lo);
        pci_write_config_dword(to_pci_dev(dev->dev), NTB_NVME_MSIX_TBL_ENTRY_HI, dword[1]);
#endif

        read_msix_cap(dev, __func__, __LINE__); // KW_DB
        
        dev_info(dev->ctrl.device, "%s() Here (Ln %d)\n", __func__, __LINE__); // KW_DB
        msix_table = map_msix_table(dev, msix_table_offset, msix_table_size);
        if (msix_table)
        {
            dma_addr_t addr = dev->dnb_addr + ((widx+1) * SZ_32K) + offset;

            lo_hi_writeq(addr, msix_table);
            iounmap(msix_table);
            dev_info(dev->ctrl.device, "%s() Wrote addr[0x%llX] to MSI-X table (Ln %d)\n",
                __func__, addr, __LINE__); // KW_DB
        }
        else
        {
            dev_err(dev->ctrl.device, "%s() Failed to map msix_table (Ln %d)\n", __func__, __LINE__); // KW_DB
        }
        read_msix_cap(dev, __func__, __LINE__); // KW_DB
    }
#endif  /* ULTRA */
    // KW_DB<-

	return result;

 free_irq:
    dev_info(dev->ctrl.device, "%s() Calling pci_free_irq_vectors() (Ln %d)\n", __func__, __LINE__); // KW_DB
	pci_free_irq_vectors(pdev);
 disable:
	pci_disable_device(pdev);
	return result;
}

static void nvme_dev_unmap(struct nvme_dev *dev)
{
	if (dev->bar)
		iounmap(dev->bar);
	pci_release_mem_regions(to_pci_dev(dev->dev));
}

static bool nvme_pci_ctrl_is_dead(struct nvme_dev *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	u32 csts;

	if (!pci_is_enabled(pdev) || !pci_device_is_present(pdev))
		return true;
	if (pdev->error_state != pci_channel_io_normal)
		return true;

	csts = readl(dev->bar + NVME_REG_CSTS);
    dev_info(dev->ctrl.device, "%s() NVME_REG_CSTS[0x%X] (Ln %d)\n", __func__, csts, __LINE__); // KW_DB
	return (csts & NVME_CSTS_CFS) || !(csts & NVME_CSTS_RDY);
}

static void nvme_dev_disable(struct nvme_dev *dev, bool shutdown)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);
	bool dead;

	mutex_lock(&dev->shutdown_lock);
    // KW_DB->
    #if 0   // The following not really working for us
    if (ntb_temp_holder)
        ntb_unregister_client(&nvme_ntb_client);
    #endif

    ///read_msix_vector(dev, 0);

    // KW_DB<-
	dead = nvme_pci_ctrl_is_dead(dev);
	if (dev->ctrl.state == NVME_CTRL_LIVE ||
	    dev->ctrl.state == NVME_CTRL_RESETTING) {
		if (pci_is_enabled(pdev))
			nvme_start_freeze(&dev->ctrl);
		/*
		 * Give the controller a chance to complete all entered requests
		 * if doing a safe shutdown.
		 */
		if (!dead && shutdown)
			nvme_wait_freeze_timeout(&dev->ctrl, NVME_IO_TIMEOUT);
	}

	nvme_quiesce_io_queues(&dev->ctrl);

	if (!dead && dev->ctrl.queue_count > 0) {
		nvme_delete_io_queues(dev);
		nvme_disable_ctrl(&dev->ctrl, shutdown);
		nvme_poll_irqdisable(&dev->queues[0]);
	}
	nvme_suspend_io_queues(dev);
	nvme_suspend_queue(dev, 0);
    dev_info(dev->ctrl.device, "%s() Calling pci_free_irq_vectors() (Ln %d)\n", __func__, __LINE__); // KW_DB
	pci_free_irq_vectors(pdev);
	if (pci_is_enabled(pdev))
		pci_disable_device(pdev);
	nvme_reap_pending_cqes(dev);

	nvme_cancel_tagset(&dev->ctrl);
	nvme_cancel_admin_tagset(&dev->ctrl);

	/*
	 * The driver will not be starting up queues again if shutting down so
	 * must flush all entered requests to their failed completion to avoid
	 * deadlocking blk-mq hot-cpu notifier.
	 */
	if (shutdown) {
		nvme_unquiesce_io_queues(&dev->ctrl);
		if (dev->ctrl.admin_q && !blk_queue_dying(dev->ctrl.admin_q))
			nvme_unquiesce_admin_queue(&dev->ctrl);
	}
	mutex_unlock(&dev->shutdown_lock);
}

static int nvme_disable_prepare_reset(struct nvme_dev *dev, bool shutdown)
{
	if (!nvme_wait_reset(&dev->ctrl))
		return -EBUSY;
	nvme_dev_disable(dev, shutdown);
	return 0;
}

static int nvme_setup_prp_pools(struct nvme_dev *dev)
{
	dev->prp_page_pool = dma_pool_create("prp list page", dev->dev,
						NVME_CTRL_PAGE_SIZE,
						NVME_CTRL_PAGE_SIZE, 0);
	if (!dev->prp_page_pool)
		return -ENOMEM;

	/* Optimisation for I/Os between 4k and 128k */
	dev->prp_small_pool = dma_pool_create("prp list 256", dev->dev,
						256, 256, 0);
	if (!dev->prp_small_pool) {
		dma_pool_destroy(dev->prp_page_pool);
		return -ENOMEM;
	}
	return 0;
}

static void nvme_release_prp_pools(struct nvme_dev *dev)
{
	dma_pool_destroy(dev->prp_page_pool);
	dma_pool_destroy(dev->prp_small_pool);
}

static int nvme_pci_alloc_iod_mempool(struct nvme_dev *dev)
{
	size_t alloc_size = sizeof(struct scatterlist) * NVME_MAX_SEGS;

	dev->iod_mempool = mempool_create_node(1,
			mempool_kmalloc, mempool_kfree,
			(void *)alloc_size, GFP_KERNEL,
			dev_to_node(dev->dev));
	if (!dev->iod_mempool)
		return -ENOMEM;
	return 0;
}

static void nvme_free_tagset(struct nvme_dev *dev)
{
	if (dev->tagset.tags)
		nvme_remove_io_tag_set(&dev->ctrl);
	dev->ctrl.tagset = NULL;
}

/* pairs with nvme_pci_alloc_dev */
static void nvme_pci_free_ctrl(struct nvme_ctrl *ctrl)
{
	struct nvme_dev *dev = to_nvme_dev(ctrl);

	nvme_free_tagset(dev);
	put_device(dev->dev);
	kfree(dev->queues);
	kfree(dev);
}

static void nvme_reset_work(struct work_struct *work)
{
	struct nvme_dev *dev =
		container_of(work, struct nvme_dev, ctrl.reset_work);
	bool was_suspend = !!(dev->ctrl.ctrl_config & NVME_CC_SHN_NORMAL);
	int result;

	if (dev->ctrl.state != NVME_CTRL_RESETTING) {
		dev_warn(dev->ctrl.device, "ctrl state %d is not RESETTING\n",
			 dev->ctrl.state);
		result = -ENODEV;
		goto out;
	}

	/*
	 * If we're called to reset a live controller first shut it down before
	 * moving on.
	 */
	if (dev->ctrl.ctrl_config & NVME_CC_ENABLE)
		nvme_dev_disable(dev, false);
	nvme_sync_queues(&dev->ctrl);

	mutex_lock(&dev->shutdown_lock);
	result = nvme_pci_enable(dev);
	if (result)
		goto out_unlock;
	nvme_unquiesce_admin_queue(&dev->ctrl);
	mutex_unlock(&dev->shutdown_lock);

	/*
	 * Introduce CONNECTING state from nvme-fc/rdma transports to mark the
	 * initializing procedure here.
	 */
	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_CONNECTING)) {
		dev_warn(dev->ctrl.device,
			"failed to mark controller CONNECTING\n");
		result = -EBUSY;
		goto out;
	}

	result = nvme_init_ctrl_finish(&dev->ctrl, was_suspend);
	if (result)
		goto out;

	nvme_dbbuf_dma_alloc(dev);

	result = nvme_setup_host_mem(dev);
	if (result < 0)
		goto out;

	result = nvme_setup_io_queues(dev);
	if (result)
		goto out;

	/*
	 * Freeze and update the number of I/O queues as thos might have
	 * changed.  If there are no I/O queues left after this reset, keep the
	 * controller around but remove all namespaces.
	 */
	if (dev->online_queues > 1) {
		nvme_unquiesce_io_queues(&dev->ctrl);
		nvme_wait_freeze(&dev->ctrl);
		nvme_pci_update_nr_queues(dev);
		nvme_dbbuf_set(dev);
		nvme_unfreeze(&dev->ctrl);
	} else {
		dev_warn(dev->ctrl.device, "IO queues lost\n");
		nvme_mark_namespaces_dead(&dev->ctrl);
		nvme_unquiesce_io_queues(&dev->ctrl);
		nvme_remove_namespaces(&dev->ctrl);
		nvme_free_tagset(dev);
	}

	/*
	 * If only admin queue live, keep it to do further investigation or
	 * recovery.
	 */
	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_LIVE)) {
		dev_warn(dev->ctrl.device,
			"failed to mark controller live state\n");
		result = -ENODEV;
		goto out;
	}

	nvme_start_ctrl(&dev->ctrl);
	return;

 out_unlock:
	mutex_unlock(&dev->shutdown_lock);
 out:
	/*
	 * Set state to deleting now to avoid blocking nvme_wait_reset(), which
	 * may be holding this pci_dev's device lock.
	 */
	dev_warn(dev->ctrl.device, "Disabling device after reset failure: %d\n",
		 result);
	nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
	nvme_dev_disable(dev, true);
	nvme_sync_queues(&dev->ctrl);
	nvme_mark_namespaces_dead(&dev->ctrl);
	nvme_unquiesce_io_queues(&dev->ctrl);
	nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DEAD);
}

static int nvme_pci_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	*val = readl(to_nvme_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_pci_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	writel(val, to_nvme_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_pci_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	*val = lo_hi_readq(to_nvme_dev(ctrl)->bar + off);
	return 0;
}

static int nvme_pci_get_address(struct nvme_ctrl *ctrl, char *buf, int size)
{
	struct pci_dev *pdev = to_pci_dev(to_nvme_dev(ctrl)->dev);

	return snprintf(buf, size, "%s\n", dev_name(&pdev->dev));
}

static void nvme_pci_print_device_info(struct nvme_ctrl *ctrl)
{
	struct pci_dev *pdev = to_pci_dev(to_nvme_dev(ctrl)->dev);
	struct nvme_subsystem *subsys = ctrl->subsys;

	dev_err(ctrl->device,
		"VID:DID %04x:%04x model:%.*s firmware:%.*s\n",
		pdev->vendor, pdev->device,
		nvme_strlen(subsys->model, sizeof(subsys->model)),
		subsys->model, nvme_strlen(subsys->firmware_rev,
					   sizeof(subsys->firmware_rev)),
		subsys->firmware_rev);
}

static bool nvme_pci_supports_pci_p2pdma(struct nvme_ctrl *ctrl)
{
	struct nvme_dev *dev = to_nvme_dev(ctrl);

	return dma_pci_p2pdma_supported(dev->dev);
}

static const struct nvme_ctrl_ops nvme_pci_ctrl_ops = {
	.name			= "pcie",
	.module			= THIS_MODULE,
	.flags			= NVME_F_METADATA_SUPPORTED,
	.dev_attr_groups	= nvme_pci_dev_attr_groups,
	.reg_read32		= nvme_pci_reg_read32,
	.reg_write32		= nvme_pci_reg_write32,
	.reg_read64		= nvme_pci_reg_read64,
	.free_ctrl		= nvme_pci_free_ctrl,
	.submit_async_event	= nvme_pci_submit_async_event,
	.get_address		= nvme_pci_get_address,
	.print_device_info	= nvme_pci_print_device_info,
	.supports_pci_p2pdma	= nvme_pci_supports_pci_p2pdma,
};

static int nvme_dev_map(struct nvme_dev *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev->dev);

    dev_info(dev->ctrl.device, "%s() Entered (Ln %d)\n", __func__, __LINE__); // KW_DB
	if (pci_request_mem_regions(pdev, "nvme"))
		return -ENODEV;

    dev_info(dev->ctrl.device, "%s() Calling nvme_remap_bar() with size[0x%lX] (Ln %d)\n",
        __func__, (unsigned long) (NVME_REG_DBS + 4096), __LINE__); // KW_DB
	if (nvme_remap_bar(dev, NVME_REG_DBS + 4096))
		goto release;

	return 0;
  release:
	pci_release_mem_regions(pdev);
	return -ENODEV;
}

static unsigned long check_vendor_combination_bug(struct pci_dev *pdev)
{
	if (pdev->vendor == 0x144d && pdev->device == 0xa802) {
		/*
		 * Several Samsung devices seem to drop off the PCIe bus
		 * randomly when APST is on and uses the deepest sleep state.
		 * This has been observed on a Samsung "SM951 NVMe SAMSUNG
		 * 256GB", a "PM951 NVMe SAMSUNG 512GB", and a "Samsung SSD
		 * 950 PRO 256GB", but it seems to be restricted to two Dell
		 * laptops.
		 */
		if (dmi_match(DMI_SYS_VENDOR, "Dell Inc.") &&
		    (dmi_match(DMI_PRODUCT_NAME, "XPS 15 9550") ||
		     dmi_match(DMI_PRODUCT_NAME, "Precision 5510")))
			return NVME_QUIRK_NO_DEEPEST_PS;
	} else if (pdev->vendor == 0x144d && pdev->device == 0xa804) {
		/*
		 * Samsung SSD 960 EVO drops off the PCIe bus after system
		 * suspend on a Ryzen board, ASUS PRIME B350M-A, as well as
		 * within few minutes after bootup on a Coffee Lake board -
		 * ASUS PRIME Z370-A
		 */
		if (dmi_match(DMI_BOARD_VENDOR, "ASUSTeK COMPUTER INC.") &&
		    (dmi_match(DMI_BOARD_NAME, "PRIME B350M-A") ||
		     dmi_match(DMI_BOARD_NAME, "PRIME Z370-A")))
			return NVME_QUIRK_NO_APST;
	} else if ((pdev->vendor == 0x144d && (pdev->device == 0xa801 ||
		    pdev->device == 0xa808 || pdev->device == 0xa809)) ||
		   (pdev->vendor == 0x1e0f && pdev->device == 0x0001)) {
		/*
		 * Forcing to use host managed nvme power settings for
		 * lowest idle power with quick resume latency on
		 * Samsung and Toshiba SSDs based on suspend behavior
		 * on Coffee Lake board for LENOVO C640
		 */
		if ((dmi_match(DMI_BOARD_VENDOR, "LENOVO")) &&
		     dmi_match(DMI_BOARD_NAME, "LNVNB161216"))
			return NVME_QUIRK_SIMPLE_SUSPEND;
	}

	return 0;
}

static struct nvme_dev *nvme_pci_alloc_dev(struct pci_dev *pdev,
		const struct pci_device_id *id)
{
	unsigned long quirks = id->driver_data;
	int node = dev_to_node(&pdev->dev);
	struct nvme_dev *dev;
	int ret = -ENOMEM;

	dev = kzalloc_node(sizeof(*dev), GFP_KERNEL, node);
	if (!dev)
		return ERR_PTR(-ENOMEM);
	INIT_WORK(&dev->ctrl.reset_work, nvme_reset_work);
	mutex_init(&dev->shutdown_lock);

	dev->nr_write_queues = write_queues;
	dev->nr_poll_queues = poll_queues;
	dev->nr_allocated_queues = nvme_max_io_queues(dev) + 1;
	dev->queues = kcalloc_node(dev->nr_allocated_queues,
			sizeof(struct nvme_queue), GFP_KERNEL, node);
	if (!dev->queues)
		goto out_free_dev;

	dev->dev = get_device(&pdev->dev);

	quirks |= check_vendor_combination_bug(pdev);
	if (!noacpi && acpi_storage_d3(&pdev->dev)) {
		/*
		 * Some systems use a bios work around to ask for D3 on
		 * platforms that support kernel managed suspend.
		 */
		dev_info(&pdev->dev,
			 "platform quirk: setting simple suspend\n");
		quirks |= NVME_QUIRK_SIMPLE_SUSPEND;
	}
    dev_info(dev->ctrl.device, "%s() Here(Ln %d)\n", __func__, __LINE__); // KW_DB
	ret = nvme_init_ctrl(&dev->ctrl, &pdev->dev, &nvme_pci_ctrl_ops,
			     quirks);
	if (ret)
		goto out_put_device;

    // KW_DB_20241211->
    #if 1   // Tries to limit DMA mapping to lower 32-bit addresses.
        dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
    #else
    // KW_DB_20241211<-
	if (dev->ctrl.quirks & NVME_QUIRK_DMA_ADDRESS_BITS_48)
		dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(48));
	else
		dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    #endif  // KW_DB_20241211

	dma_set_min_align_mask(&pdev->dev, NVME_CTRL_PAGE_SIZE - 1);
	dma_set_max_seg_size(&pdev->dev, 0xffffffff);

    dev_info(dev->ctrl.device, "%s() Here(Ln %d)\n", __func__, __LINE__); // KW_DB
	/*
	 * Limit the max command size to prevent iod->sg allocations going
	 * over a single page.
	 */
	dev->ctrl.max_hw_sectors = min_t(u32,
		NVME_MAX_KB_SZ << 1, dma_opt_mapping_size(&pdev->dev) >> 9);
	dev->ctrl.max_segments = NVME_MAX_SEGS;

	/*
	 * There is no support for SGLs for metadata (yet), so we are limited to
	 * a single integrity segment for the separate metadata pointer.
	 */
	dev->ctrl.max_integrity_segments = 1;
    dev->free_widx = SWITCHTEC_WIDX_BASE;   // KW_DB
	return dev;

out_put_device:
	put_device(dev->dev);
	kfree(dev->queues);
out_free_dev:
	kfree(dev);
	return ERR_PTR(ret);
}

static int nvme_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct nvme_dev *dev;
	int result = -ENOMEM;

    pr_info("%s() Calling nvme_dev_map() (Ln %d)\n", __func__, __LINE__); // KW_DB
	dev = nvme_pci_alloc_dev(pdev, id);
	if (IS_ERR(dev))
		return PTR_ERR(dev);

    dev_info(dev->ctrl.device, "%s() Calling nvme_dev_map() (Ln %d)\n", __func__, __LINE__); // KW_DB
	result = nvme_dev_map(dev);
	if (result)
		goto out_uninit_ctrl;

	result = nvme_setup_prp_pools(dev);
	if (result)
		goto out_dev_unmap;

	result = nvme_pci_alloc_iod_mempool(dev);
	if (result)
		goto out_release_prp_pools;

	dev_info(dev->ctrl.device, "pci function %s\n", dev_name(&pdev->dev));

	result = nvme_pci_enable(dev);
	if (result)
		goto out_release_iod_mempool;

    dev_info(dev->ctrl.device, "%s() Here (Ln %d)\n", __func__, __LINE__); // KW_DB
    ///dev_info(dev->ctrl.device, "Calling read_msix_vector()\n"); // KW_DB
    ///read_msix_vector(dev, 0);                                   // KW_DB

	result = nvme_alloc_admin_tag_set(&dev->ctrl, &dev->admin_tagset,
				&nvme_mq_admin_ops, sizeof(struct nvme_iod));
	if (result)
		goto out_disable;

    ///dev_info(dev->ctrl.device, "Calling read_msix_vector()\n"); // KW_DB
    ///read_msix_vector(dev, 0);                                   // KW_DB

	/*
	 * Mark the controller as connecting before sending admin commands to
	 * allow the timeout handler to do the right thing.
	 */
	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_CONNECTING)) {
		dev_warn(dev->ctrl.device,
			"failed to mark controller CONNECTING\n");
		result = -EBUSY;
		goto out_disable;
	}

    ///dev_info(dev->ctrl.device, "Calling read_msix_vector()\n"); // KW_DB
    ///read_msix_vector(dev, 0);                                   // KW_DB

	result = nvme_init_ctrl_finish(&dev->ctrl, false);
	if (result)
		goto out_disable;

	nvme_dbbuf_dma_alloc(dev);

	result = nvme_setup_host_mem(dev);
	if (result < 0)
		goto out_disable;

	result = nvme_setup_io_queues(dev);
	if (result)
		goto out_disable;

    dump_msix_table_entry(dev, 1, 4, __LINE__);  // KW_DB
	if (dev->online_queues > 1) {
		nvme_alloc_io_tag_set(&dev->ctrl, &dev->tagset, &nvme_mq_ops,
				nvme_pci_nr_maps(dev), sizeof(struct nvme_iod));
		nvme_dbbuf_set(dev);
	}

	if (!dev->ctrl.tagset)
		dev_warn(dev->ctrl.device, "IO queues not created\n");

	if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_LIVE)) {
		dev_warn(dev->ctrl.device,
			"failed to mark controller live state\n");
		result = -ENODEV;
		goto out_disable;
	}

	pci_set_drvdata(pdev, dev);

	nvme_start_ctrl(&dev->ctrl);
	nvme_put_ctrl(&dev->ctrl);
    dump_msix_table_entry(dev, 1, 4, __LINE__);  // KW_DB
	flush_work(&dev->ctrl.scan_work);
	return 0;

out_disable:
	nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
	nvme_dev_disable(dev, true);
	nvme_free_host_mem(dev);
	nvme_dev_remove_admin(dev);
	nvme_dbbuf_dma_free(dev);
	nvme_free_queues(dev, 0);
out_release_iod_mempool:
	mempool_destroy(dev->iod_mempool);
out_release_prp_pools:
	nvme_release_prp_pools(dev);
out_dev_unmap:
	nvme_dev_unmap(dev);
out_uninit_ctrl:
	nvme_uninit_ctrl(&dev->ctrl);
	nvme_put_ctrl(&dev->ctrl);
	return result;
}

static void nvme_reset_prepare(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	/*
	 * We don't need to check the return value from waiting for the reset
	 * state as pci_dev device lock is held, making it impossible to race
	 * with ->remove().
	 */
	nvme_disable_prepare_reset(dev, false);
	nvme_sync_queues(&dev->ctrl);
}

static void nvme_reset_done(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	if (!nvme_try_sched_reset(&dev->ctrl))
		flush_work(&dev->ctrl.reset_work);
}

static void nvme_shutdown(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	nvme_disable_prepare_reset(dev, true);
}

/*
 * The driver's remove may be called on a device in a partially initialized
 * state. This function must not have any dependencies on the device state in
 * order to proceed.
 */
static void nvme_remove(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DELETING);
	pci_set_drvdata(pdev, NULL);

	if (!pci_device_is_present(pdev)) {
		nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_DEAD);
		nvme_dev_disable(dev, true);
	}

	flush_work(&dev->ctrl.reset_work);
	nvme_stop_ctrl(&dev->ctrl);
	nvme_remove_namespaces(&dev->ctrl);
	nvme_dev_disable(dev, true);
	nvme_free_host_mem(dev);
	nvme_dev_remove_admin(dev);
	nvme_dbbuf_dma_free(dev);
	nvme_free_queues(dev, 0);
	mempool_destroy(dev->iod_mempool);
	nvme_release_prp_pools(dev);
	nvme_dev_unmap(dev);
	nvme_uninit_ctrl(&dev->ctrl);
}

#ifdef CONFIG_PM_SLEEP
static int nvme_get_power_state(struct nvme_ctrl *ctrl, u32 *ps)
{
	return nvme_get_features(ctrl, NVME_FEAT_POWER_MGMT, 0, NULL, 0, ps);
}

static int nvme_set_power_state(struct nvme_ctrl *ctrl, u32 ps)
{
	return nvme_set_features(ctrl, NVME_FEAT_POWER_MGMT, ps, NULL, 0, NULL);
}

static int nvme_resume(struct device *dev)
{
	struct nvme_dev *ndev = pci_get_drvdata(to_pci_dev(dev));
	struct nvme_ctrl *ctrl = &ndev->ctrl;

	if (ndev->last_ps == U32_MAX ||
	    nvme_set_power_state(ctrl, ndev->last_ps) != 0)
		goto reset;
	if (ctrl->hmpre && nvme_setup_host_mem(ndev))
		goto reset;

	return 0;
reset:
	return nvme_try_sched_reset(ctrl);
}

static int nvme_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct nvme_dev *ndev = pci_get_drvdata(pdev);
	struct nvme_ctrl *ctrl = &ndev->ctrl;
	int ret = -EBUSY;

	ndev->last_ps = U32_MAX;

	/*
	 * The platform does not remove power for a kernel managed suspend so
	 * use host managed nvme power settings for lowest idle power if
	 * possible. This should have quicker resume latency than a full device
	 * shutdown.  But if the firmware is involved after the suspend or the
	 * device does not support any non-default power states, shut down the
	 * device fully.
	 *
	 * If ASPM is not enabled for the device, shut down the device and allow
	 * the PCI bus layer to put it into D3 in order to take the PCIe link
	 * down, so as to allow the platform to achieve its minimum low-power
	 * state (which may not be possible if the link is up).
	 */
	if (pm_suspend_via_firmware() || !ctrl->npss ||
	    !pcie_aspm_enabled(pdev) ||
	    (ndev->ctrl.quirks & NVME_QUIRK_SIMPLE_SUSPEND))
		return nvme_disable_prepare_reset(ndev, true);

	nvme_start_freeze(ctrl);
	nvme_wait_freeze(ctrl);
	nvme_sync_queues(ctrl);

	if (ctrl->state != NVME_CTRL_LIVE)
		goto unfreeze;

	/*
	 * Host memory access may not be successful in a system suspend state,
	 * but the specification allows the controller to access memory in a
	 * non-operational power state.
	 */
	if (ndev->hmb) {
		ret = nvme_set_host_mem(ndev, 0);
		if (ret < 0)
			goto unfreeze;
	}

	ret = nvme_get_power_state(ctrl, &ndev->last_ps);
	if (ret < 0)
		goto unfreeze;

	/*
	 * A saved state prevents pci pm from generically controlling the
	 * device's power. If we're using protocol specific settings, we don't
	 * want pci interfering.
	 */
	pci_save_state(pdev);

	ret = nvme_set_power_state(ctrl, ctrl->npss);
	if (ret < 0)
		goto unfreeze;

	if (ret) {
		/* discard the saved state */
		pci_load_saved_state(pdev, NULL);

		/*
		 * Clearing npss forces a controller reset on resume. The
		 * correct value will be rediscovered then.
		 */
		ret = nvme_disable_prepare_reset(ndev, true);
		ctrl->npss = 0;
	}
unfreeze:
	nvme_unfreeze(ctrl);
	return ret;
}

static int nvme_simple_suspend(struct device *dev)
{
	struct nvme_dev *ndev = pci_get_drvdata(to_pci_dev(dev));

	return nvme_disable_prepare_reset(ndev, true);
}

static int nvme_simple_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct nvme_dev *ndev = pci_get_drvdata(pdev);

	return nvme_try_sched_reset(&ndev->ctrl);
}

static const struct dev_pm_ops nvme_dev_pm_ops = {
	.suspend	= nvme_suspend,
	.resume		= nvme_resume,
	.freeze		= nvme_simple_suspend,
	.thaw		= nvme_simple_resume,
	.poweroff	= nvme_simple_suspend,
	.restore	= nvme_simple_resume,
};
#endif /* CONFIG_PM_SLEEP */

static pci_ers_result_t nvme_error_detected(struct pci_dev *pdev,
						pci_channel_state_t state)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	/*
	 * A frozen channel requires a reset. When detected, this method will
	 * shutdown the controller to quiesce. The controller will be restarted
	 * after the slot reset through driver's slot_reset callback.
	 */
	switch (state) {
	case pci_channel_io_normal:
		return PCI_ERS_RESULT_CAN_RECOVER;
	case pci_channel_io_frozen:
		dev_warn(dev->ctrl.device,
			"frozen state error detected, reset controller\n");
		if (!nvme_change_ctrl_state(&dev->ctrl, NVME_CTRL_RESETTING)) {
			nvme_dev_disable(dev, true);
			return PCI_ERS_RESULT_DISCONNECT;
		}
		nvme_dev_disable(dev, false);
		return PCI_ERS_RESULT_NEED_RESET;
	case pci_channel_io_perm_failure:
		dev_warn(dev->ctrl.device,
			"failure state error detected, request disconnect\n");
		return PCI_ERS_RESULT_DISCONNECT;
	}
	return PCI_ERS_RESULT_NEED_RESET;
}

static pci_ers_result_t nvme_slot_reset(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	dev_info(dev->ctrl.device, "restart after slot reset\n");
	pci_restore_state(pdev);
	if (!nvme_try_sched_reset(&dev->ctrl))
		nvme_unquiesce_io_queues(&dev->ctrl);
	return PCI_ERS_RESULT_RECOVERED;
}

static void nvme_error_resume(struct pci_dev *pdev)
{
	struct nvme_dev *dev = pci_get_drvdata(pdev);

	flush_work(&dev->ctrl.reset_work);
}

static const struct pci_error_handlers nvme_err_handler = {
	.error_detected	= nvme_error_detected,
	.slot_reset	= nvme_slot_reset,
	.resume		= nvme_error_resume,
	.reset_prepare	= nvme_reset_prepare,
	.reset_done	= nvme_reset_done,
};

static const struct pci_device_id nvme_id_table[] = {
	{ PCI_VDEVICE(INTEL, 0x0953),	/* Intel 750/P3500/P3600/P3700 */
		.driver_data = NVME_QUIRK_STRIPE_SIZE |
				NVME_QUIRK_DEALLOCATE_ZEROES, },
	{ PCI_VDEVICE(INTEL, 0x0a53),	/* Intel P3520 */
		.driver_data = NVME_QUIRK_STRIPE_SIZE |
				NVME_QUIRK_DEALLOCATE_ZEROES, },
	{ PCI_VDEVICE(INTEL, 0x0a54),	/* Intel P4500/P4600 */
		.driver_data = NVME_QUIRK_STRIPE_SIZE |
				NVME_QUIRK_DEALLOCATE_ZEROES |
				NVME_QUIRK_IGNORE_DEV_SUBNQN |
				NVME_QUIRK_BOGUS_NID, },
	{ PCI_VDEVICE(INTEL, 0x0a55),	/* Dell Express Flash P4600 */
		.driver_data = NVME_QUIRK_STRIPE_SIZE |
				NVME_QUIRK_DEALLOCATE_ZEROES, },
	{ PCI_VDEVICE(INTEL, 0xf1a5),	/* Intel 600P/P3100 */
		.driver_data = NVME_QUIRK_NO_DEEPEST_PS |
				NVME_QUIRK_MEDIUM_PRIO_SQ |
				NVME_QUIRK_NO_TEMP_THRESH_CHANGE |
				NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_VDEVICE(INTEL, 0xf1a6),	/* Intel 760p/Pro 7600p */
		.driver_data = NVME_QUIRK_IGNORE_DEV_SUBNQN, },
	{ PCI_VDEVICE(INTEL, 0x5845),	/* Qemu emulated controller */
		.driver_data = NVME_QUIRK_IDENTIFY_CNS |
				NVME_QUIRK_DISABLE_WRITE_ZEROES |
				NVME_QUIRK_BOGUS_NID, },
	{ PCI_VDEVICE(REDHAT, 0x0010),	/* Qemu emulated controller */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x126f, 0x2263),	/* Silicon Motion unidentified */
		.driver_data = NVME_QUIRK_NO_NS_DESC_LIST |
				NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1bb1, 0x0100),   /* Seagate Nytro Flash Storage */
		.driver_data = NVME_QUIRK_DELAY_BEFORE_CHK_RDY |
				NVME_QUIRK_NO_NS_DESC_LIST, },
	{ PCI_DEVICE(0x1c58, 0x0003),	/* HGST adapter */
		.driver_data = NVME_QUIRK_DELAY_BEFORE_CHK_RDY, },
	{ PCI_DEVICE(0x1c58, 0x0023),	/* WDC SN200 adapter */
		.driver_data = NVME_QUIRK_DELAY_BEFORE_CHK_RDY, },
	{ PCI_DEVICE(0x1c5f, 0x0540),	/* Memblaze Pblaze4 adapter */
		.driver_data = NVME_QUIRK_DELAY_BEFORE_CHK_RDY, },
	{ PCI_DEVICE(0x144d, 0xa821),   /* Samsung PM1725 */
		.driver_data = NVME_QUIRK_DELAY_BEFORE_CHK_RDY, },
	{ PCI_DEVICE(0x144d, 0xa822),   /* Samsung PM1725a */
		.driver_data = NVME_QUIRK_DELAY_BEFORE_CHK_RDY |
				NVME_QUIRK_DISABLE_WRITE_ZEROES|
				NVME_QUIRK_IGNORE_DEV_SUBNQN, },
	{ PCI_DEVICE(0x1987, 0x5012),	/* Phison E12 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1987, 0x5016),	/* Phison E16 */
		.driver_data = NVME_QUIRK_IGNORE_DEV_SUBNQN |
				NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1987, 0x5019),  /* phison E19 */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x1987, 0x5021),   /* Phison E21 */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x1b4b, 0x1092),	/* Lexar 256 GB SSD */
		.driver_data = NVME_QUIRK_NO_NS_DESC_LIST |
				NVME_QUIRK_IGNORE_DEV_SUBNQN, },
	{ PCI_DEVICE(0x1cc1, 0x33f8),   /* ADATA IM2P33F8ABR1 1 TB */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x10ec, 0x5762),   /* ADATA SX6000LNP */
		.driver_data = NVME_QUIRK_IGNORE_DEV_SUBNQN |
				NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x10ec, 0x5763),  /* ADATA SX6000PNP */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1cc1, 0x8201),   /* ADATA SX8200PNP 512GB */
		.driver_data = NVME_QUIRK_NO_DEEPEST_PS |
				NVME_QUIRK_IGNORE_DEV_SUBNQN, },
	 { PCI_DEVICE(0x1344, 0x5407), /* Micron Technology Inc NVMe SSD */
		.driver_data = NVME_QUIRK_IGNORE_DEV_SUBNQN },
	 { PCI_DEVICE(0x1344, 0x6001),   /* Micron Nitro NVMe */
		 .driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1c5c, 0x1504),   /* SK Hynix PC400 */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x1c5c, 0x174a),   /* SK Hynix P31 SSD */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x15b7, 0x2001),   /*  Sandisk Skyhawk */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x1d97, 0x2263),   /* SPCC */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x144d, 0xa80b),   /* Samsung PM9B1 256G and 512G */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES |
				NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x144d, 0xa809),   /* Samsung MZALQ256HBJD 256G */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x144d, 0xa802),   /* Samsung SM953 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1cc4, 0x6303),   /* UMIS RPJTJ512MGE1QDY 512G */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x1cc4, 0x6302),   /* UMIS RPJTJ256MGE1QDY 256G */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x2646, 0x2262),   /* KINGSTON SKC2000 NVMe SSD */
		.driver_data = NVME_QUIRK_NO_DEEPEST_PS, },
	{ PCI_DEVICE(0x2646, 0x2263),   /* KINGSTON A2000 NVMe SSD  */
		.driver_data = NVME_QUIRK_NO_DEEPEST_PS, },
	{ PCI_DEVICE(0x2646, 0x5013),   /* Kingston KC3000, Kingston FURY Renegade */
		.driver_data = NVME_QUIRK_NO_SECONDARY_TEMP_THRESH, },
	{ PCI_DEVICE(0x2646, 0x5018),   /* KINGSTON OM8SFP4xxxxP OS21012 NVMe SSD */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x2646, 0x5016),   /* KINGSTON OM3PGP4xxxxP OS21011 NVMe SSD */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x2646, 0x501A),   /* KINGSTON OM8PGP4xxxxP OS21005 NVMe SSD */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x2646, 0x501B),   /* KINGSTON OM8PGP4xxxxQ OS21005 NVMe SSD */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x2646, 0x501E),   /* KINGSTON OM3PGP4xxxxQ OS21011 NVMe SSD */
		.driver_data = NVME_QUIRK_DISABLE_WRITE_ZEROES, },
	{ PCI_DEVICE(0x1f40, 0x1202),   /* Netac Technologies Co. NV3000 NVMe SSD */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1f40, 0x5236),   /* Netac Technologies Co. NV7000 NVMe SSD */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1e4B, 0x1001),   /* MAXIO MAP1001 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1e4B, 0x1002),   /* MAXIO MAP1002 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1e4B, 0x1202),   /* MAXIO MAP1202 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1e4B, 0x1602),   /* MAXIO MAP1602 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1cc1, 0x5350),   /* ADATA XPG GAMMIX S50 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1dbe, 0x5236),   /* ADATA XPG GAMMIX S70 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1e49, 0x0021),   /* ZHITAI TiPro5000 NVMe SSD */
		.driver_data = NVME_QUIRK_NO_DEEPEST_PS, },
	{ PCI_DEVICE(0x1e49, 0x0041),   /* ZHITAI TiPro7000 NVMe SSD */
		.driver_data = NVME_QUIRK_NO_DEEPEST_PS, },
	{ PCI_DEVICE(0xc0a9, 0x540a),   /* Crucial P2 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1d97, 0x2263), /* Lexar NM610 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1d97, 0x1d97), /* Lexar NM620 */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1d97, 0x2269), /* Lexar NM760 */
		.driver_data = NVME_QUIRK_BOGUS_NID |
				NVME_QUIRK_IGNORE_DEV_SUBNQN, },
	{ PCI_DEVICE(0x10ec, 0x5763), /* TEAMGROUP T-FORCE CARDEA ZERO Z330 SSD */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x1e4b, 0x1602), /* HS-SSD-FUTURE 2048G  */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(0x10ec, 0x5765), /* TEAMGROUP MP33 2TB SSD */
		.driver_data = NVME_QUIRK_BOGUS_NID, },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMAZON, 0x0061),
		.driver_data = NVME_QUIRK_DMA_ADDRESS_BITS_48, },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMAZON, 0x0065),
		.driver_data = NVME_QUIRK_DMA_ADDRESS_BITS_48, },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMAZON, 0x8061),
		.driver_data = NVME_QUIRK_DMA_ADDRESS_BITS_48, },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMAZON, 0xcd00),
		.driver_data = NVME_QUIRK_DMA_ADDRESS_BITS_48, },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMAZON, 0xcd01),
		.driver_data = NVME_QUIRK_DMA_ADDRESS_BITS_48, },
	{ PCI_DEVICE(PCI_VENDOR_ID_AMAZON, 0xcd02),
		.driver_data = NVME_QUIRK_DMA_ADDRESS_BITS_48, },
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, 0x2001),
		.driver_data = NVME_QUIRK_SINGLE_VECTOR },
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, 0x2003) },
	{ PCI_DEVICE(PCI_VENDOR_ID_APPLE, 0x2005),
		.driver_data = NVME_QUIRK_SINGLE_VECTOR |
				NVME_QUIRK_128_BYTES_SQES |
				NVME_QUIRK_SHARED_TAGS |
				NVME_QUIRK_SKIP_CID_GEN |
				NVME_QUIRK_IDENTIFY_CNS },
	{ PCI_DEVICE_CLASS(PCI_CLASS_STORAGE_EXPRESS, 0xffffff) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, nvme_id_table);

static struct pci_driver nvme_driver = {
	.name		= "nvme",
	.id_table	= nvme_id_table,
	.probe		= nvme_probe,
	.remove		= nvme_remove,
	.shutdown	= nvme_shutdown,
	.driver		= {
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
#ifdef CONFIG_PM_SLEEP
		.pm		= &nvme_dev_pm_ops,
#endif
	},
	.sriov_configure = pci_sriov_configure_simple,
	.err_handler	= &nvme_err_handler,
};

static int __init nvme_init(void)
{
    ///int result; // KW_DB

	BUILD_BUG_ON(sizeof(struct nvme_create_cq) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_create_sq) != 64);
	BUILD_BUG_ON(sizeof(struct nvme_delete_queue) != 64);
	BUILD_BUG_ON(IRQ_AFFINITY_MAX_SETS < 2);
	BUILD_BUG_ON(NVME_MAX_SEGS > SGES_PER_PAGE);
	BUILD_BUG_ON(sizeof(struct scatterlist) * NVME_MAX_SEGS > PAGE_SIZE);
	BUILD_BUG_ON(nvme_pci_npages_prp() > NVME_MAX_NR_ALLOCATIONS);

    // KW_DB->
    #if 0   // The following not really working for us
    result = ntb_register_client(&nvme_ntb_client);
    if (result)
        pr_err("%s() ntb_register_client() result[%d] (Ln %d)\n", __func__, result,__LINE__);
    #endif
    // KW_DB<-

	return pci_register_driver(&nvme_driver);
}

static void __exit nvme_exit(void)
{
	pci_unregister_driver(&nvme_driver);
	flush_workqueue(nvme_wq);
}

MODULE_AUTHOR("Matthew Wilcox <willy@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
module_init(nvme_init);
module_exit(nvme_exit);
