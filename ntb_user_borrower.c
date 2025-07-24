#include <linux/module.h>   /* Needed by all modules */
#include <linux/kernel.h>   /* Needed for KERN_INFO */
#include <linux/sysfs.h>
#include <linux/pci.h>

/* LINUX_VERSION_CODE */
#include <linux/version.h>

/* dma_map_ops */
#include <linux/dma-map-ops.h>

/* procfs */
#include <linux/proc_fs.h>
#include <linux/ntb.h>
#include "ntb_device_lending.h"
#include "ntb_nvme_inf.h"

#define MY_DEVFN    (4)

#define MAX_NVME_Q_PAIRS    (32)    // TODO: Revise.

#define DMAP_LEN            (MAX_NVME_Q_PAIRS*2 + 1)


#define INTERFERE_BAR_WRITE (1) // KW_DB_NEW


/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
typedef struct sk_vdev sk_vdev_t;

typedef int (*vcfgspace_func_read_t)(sk_vdev_t *vdev, void *bus, uint32_t devfn,
                                     int where, int size, uint32_t *val);
typedef int (*vcfgspace_func_write_t)(sk_vdev_t *vdev, void *bus, uint32_t devfn,
                                      int where, int size, uint32_t val);

struct sk_vdev {
    uint32_t                    devfn;
    struct pci_dev            * pdev;
    struct pci_bus            * parent_bus;
    uint32_t                    type;
    vcfgspace_func_read_t       vcfgspace_read;
    vcfgspace_func_write_t      vcfgspace_write;
    resource_size_t             start_orig;
	resource_size_t             end_orig;
    void                      * user_data;
    int                         dmap_idx;   /* value = vacant slot */
/* [cpu addr] [dma_handle] */
    dma_addr_t                * dmap[DMAP_LEN][2];
    // struct device             * dev;
    // const struct dma_map_ops  * saved_dma_mapping_ops;
    u32                         msix_table_offset;
    u32                         msix_table_size;
    u32                         msix_widx[MAX_NVME_Q_PAIRS+1];
    u32                         msix_addr_lo[MAX_NVME_Q_PAIRS+1];
    u32                         msix_addr_hi[MAX_NVME_Q_PAIRS+1];
    u32                         sq_widx[MAX_NVME_Q_PAIRS+1];
    u32                         sq_addr_lo[MAX_NVME_Q_PAIRS+1];
    u32                         sq_addr_hi[MAX_NVME_Q_PAIRS+1];
    u32                         cq_widx[MAX_NVME_Q_PAIRS+1];
    u32                         cq_addr_lo[MAX_NVME_Q_PAIRS+1];
    u32                         cq_addr_hi[MAX_NVME_Q_PAIRS+1];
    u32                         prp1_widx[MAX_NVME_Q_PAIRS+1];
    u32                         prp1_addr_lo[MAX_NVME_Q_PAIRS+1];
    u32                         prp1_addr_hi[MAX_NVME_Q_PAIRS+1];
    u32                         msi_addr_lo;
    u32                         msi_addr_hi;
    u64                         msi_redirect_addr;
    /* For xlat & size alignment */
    u32                         widx;
    resource_size_t             xlat_align;
    resource_size_t             size_align;
    resource_size_t             size_max;
};


struct sk_vdev_instance {
    volatile struct pci_ops *pci_ops_orig;
    sk_vdev_t *vdev_table[256];
    // osif_iommu_domain_t iommu_domain;
    // osif_mutex_t lock;
};
typedef struct sk_vdev_instance sk_vdev_instance_t;


sk_vdev_instance_t * vdev_instances[256];

static sk_vdev_t * sk_vdev;

static u8 config_rw_cmd_idx = 0;

# if (INTERFERE_BAR_WRITE > 0)
static bool bar_sizing = false;
static int bar_sizing_count = 0;
#endif

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/

/* DMA API functions hooks */
#define INTERCEPT_DMA_API   (1)

#define SETUP_DMA_MAPPING   (1)

#if (INTERCEPT_DMA_API == 1)

#include <linux/dma-map-ops.h>
// #include <linux/dma-mapping.h>
// #include <linux/dma-direct.h>

#if (SETUP_DMA_MAPPING == 1)
/**
 *
 */
static int borrower_add_to_mapping_table(struct device * dev,
                                         void          * cpu_addr,
                                         void          * dma_handle,
                                         size_t          size)
{
    struct pci_dev        * pdev = NULL;
    sk_vdev_instance_t    * vdev_inst = NULL;
    sk_vdev_t             * vdev = NULL;
    struct perf_ctx       * perf = NULL;
    struct perf_peer      * peer = NULL;
    // resource_size_t         xlat_align, size_align, size_max;
    // int                     ret;

    /* Alternative: container_of(dev, struct pci_dev, dev) */
    pdev = to_pci_dev(dev);

    dev_info(dev, "%s(): pdev(%p) cpu_addr(%p) dma_handle(%p) (Line #%d)\n",
        __func__, pdev, cpu_addr, dma_handle, __LINE__);

    if (!pdev)
    {
        dev_err(dev, "Fail to setup mapping: pdev is NULL!\n");
        return -1;
    }
    // vdev = container_of(pdev, sk_vdev_t, *pdev);
    // dev_info(dev, "%s(): vdev(%p)  (PCI #%d)\n", __func__, vdev, __LINE__);

    vdev_inst = vdev_instances[pdev->bus->number];
    if (!vdev_inst)
    {
        dev_err(dev, "Fail to setup mapping: vdev_inst is NULL!\n");
        return -1;
    }

    vdev = vdev_inst->vdev_table[MY_DEVFN];
    if (!vdev)
    {
        dev_err(dev, "Fail to setup mapping: vdev is NULL!\n");
        return -1;
    }

    perf = vdev->user_data;
    peer = &perf->peers[0];
#if 0
    /* Sizing things... */
    ret = ntb_mw_get_align(perf->ntb, peer->pidx, vdev->dmap_idx + 4),
                           &xlat_align, &size_align, &size_max);
    if (ret) {
        dev_err(&perf->ntb->dev, "Couldn't get pciCfgBuf restrictions\n");
        return ret;
    }
    if (size > size_max) {
        dev_err(&perf->ntb->dev, "Queue size too big %ld > %pa\n",
            size, &size_max);
        return -EINVAL;
    }
#endif

    /* Add to mapping table. */
    vdev->dmap[vdev->dmap_idx][0] = cpu_addr;
    vdev->dmap[vdev->dmap_idx][1] = dma_handle;
    vdev->dmap_idx++;

    return 0;
}

/**
 *
 */
static int borrower_remove_from_mapping_tabke(struct device * dev,
                                              void          * cpu_addr,
                                              void          * dma_handle,
                                              size_t          size)
{
    struct pci_dev        * pdev = to_pci_dev(dev);
    sk_vdev_instance_t    * vdev_inst = NULL;
    sk_vdev_t             * vdev = NULL;
    struct perf_ctx       * perf = NULL;
    struct perf_peer      * peer = NULL;
    int                     i = 0;

    dev_info(dev, "%s(): pdev(%p) cpu_addr(%p) (Line #%d)\n", __func__, pdev, cpu_addr, __LINE__);
    if (!pdev)
    {
        dev_err(dev, "Fail to remove mapping: pdev is NULL!\n");
        return -1;
    }

    vdev_inst = vdev_instances[pdev->bus->number];
    if (!vdev_inst)
    {
        dev_err(dev, "Fail to remove mapping: vdev_inst is NULL!\n");
        return -1;
    }

    vdev = vdev_inst->vdev_table[MY_DEVFN];
    if (!vdev)
    {
        dev_err(dev, "Fail to remove mapping: vdev is NULL!\n");
        return -1;
    }

    perf = vdev->user_data;
    peer = &perf->peers[0];

    for (i=0; i<DMAP_LEN; i++)
    {
        if (cpu_addr == vdev->dmap[i][0])
        {
            dma_handle = (void *) vdev->dmap[i][1];
            vdev->dmap[i][0] = NULL;
            vdev->dmap[i][1] = NULL;

            dev_info(dev, "%s(): Entry dma_handle(%p) found and cleared from table. (Ln %d) \n", __func__, dma_handle, __LINE__);
        }
    }
    return 0;
}

/**
 *
 */
static int borrower_setup_dmaAccess(struct device * dev,
                                    dma_addr_t      dma_handle,
                                    size_t          size)
{
    // struct pci_dev        * pdev = to_pci_dev(dev);
    // sk_vdev_instance_t    * vdev_inst = vdev_instances[pdev->bus->number];
    // sk_vdev_t             * vdev = vdev_inst->vdev_table[MY_DEVFN];
    // struct perf_ctx       * perf = vdev->user_data;
    // struct perf_peer      * peer = &perf->peers[0];
    // phys_addr_t             ntb_base_addr;
    int                     ret = 0;

    // ntb_base_addr = pci_resource_start(pdev, 2);



#if 0
    ret = ntb_mw_set_trans(perf->ntb, peer->pidx, vdev->dmap_idx+3, dma_handle, size);
    if (ret)
        dev_err(dev, "%s(): Failed to setup LUT for dma_handle[0x%llX]. (Ln %d) \n", __func__, dma_handle, __LINE__);
#endif
    return ret;
}

/**
 *
 */
static void genXlateDmaHandle(struct device *dev, dma_addr_t *dma_handle)
{
    struct pci_dev        * pdev = to_pci_dev(dev);
    sk_vdev_instance_t    * vdev_inst = vdev_instances[pdev->bus->number];
    sk_vdev_t             * vdev = vdev_inst->vdev_table[MY_DEVFN];
    struct perf_ctx       * perf = vdev->user_data;
    struct perf_peer      * peer = &perf->peers[0];
    dma_addr_t              new_dma_handle;

    // TODO: Use LUT_SIZE instead. E.f. send msg...
    new_dma_handle = peer->dnb_addr + SZ_32K + (vdev->dmap_idx + 3) * SZ_32K;
    dev_info(dev, "%s(): new_dma_handle[0x%llX]. (Ln %d) \n", __func__, new_dma_handle, __LINE__);
    dma_handle = (dma_addr_t  *) new_dma_handle;
}
#endif /* SETUP_DMA_MAPPING */




/*
 * Ref: https://elixir.bootlin.com/linux/v5.10.1/source/include/linux/dma-map-ops.h#L14
 *      https://elixir.bootlin.com/linux/v5.10.1/source/kernel/dma/mapping.c#L639
 */
static const struct dma_map_ops * saved_dma_mapping_ops;

/**
 *
 */
static void check_dma_ops_implementation(struct device *dev, const struct dma_map_ops *ops)
{
    int total_cnt = 0;
    int implemented_cnt = 0;

    if (ops->alloc) {
        dev_info(dev, "%s():   alloc(%p). (PCI #%d)\n", __func__, ops->alloc, __LINE__); // KW_DB
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->free) {
        dev_info(dev, "%s():   free(%p). (PCI #%d)\n", __func__, ops->free, __LINE__); // KW_DB
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->alloc_pages) {
        dev_info(dev, "%s():   alloc_pages(%p). (PCI #%d)\n", __func__, ops->alloc_pages, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->free_pages) {
        dev_info(dev, "%s():   free_pages(%p). (PCI #%d)\n", __func__, ops->free_pages, __LINE__); // KW_DB
        implemented_cnt++;
    }
    total_cnt++;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,12,00)
    if (ops->alloc_noncoherent) {
        dev_info(dev, "%s():   alloc_noncoherent(%p). (PCI #%d)\n", __func__, ops->alloc_noncoherent, __LINE__); // KW_DB
        implemented_cnt++;
    }
#else
    if (ops->alloc_noncontiguous) {
        dev_info(dev, "%s():   alloc_noncontiguous(%p). (PCI #%d)\n", __func__, ops->alloc_noncontiguous, __LINE__); // KW_DB
        implemented_cnt++;
    }
#endif
    total_cnt++;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,12,00)
    if (ops->free_noncoherent) {
        dev_info(dev, "%s():   free_noncoherent(%p). (PCI #%d)\n", __func__, ops->free_noncoherent, __LINE__); // KW_DB
        implemented_cnt++;
    }
#else
    if (ops->free_noncontiguous) {
        dev_info(dev, "%s():   free_noncontiguous(%p). (PCI #%d)\n", __func__, ops->free_noncontiguous, __LINE__); // KW_DB
        implemented_cnt++;
    }
#endif
    total_cnt++;
    if (ops->mmap) {
        dev_info(dev, "%s():   mmap(%p). (PCI #%d)\n", __func__, ops->mmap, __LINE__); // KW_DB
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->get_sgtable) {
        dev_info(dev, "%s():   get_sgtable(%p). (PCI #%d)\n", __func__, ops->get_sgtable, __LINE__); // KW_DB
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->map_page) {
        dev_info(dev, "%s():   map_page(%p). (PCI #%d)\n", __func__, ops->map_page, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->map_sg) {
        dev_info(dev, "%s():   map_sg(%p). (PCI #%d)\n", __func__, ops->map_sg, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->unmap_sg) {
        dev_info(dev, "%s():   unmap_sg(%p). (PCI #%d)\n", __func__, ops->unmap_sg, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->map_resource) {
        dev_info(dev, "%s():   map_resource(%p). (PCI #%d)\n", __func__, ops->map_resource, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->unmap_resource) {
        dev_info(dev, "%s():   unmap_resource(%p). (PCI #%d)\n", __func__, ops->unmap_resource, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->sync_single_for_cpu) {
        dev_info(dev, "%s():   sync_single_for_cpu(%p). (PCI #%d)\n", __func__, ops->sync_single_for_cpu, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->sync_single_for_device) {
        dev_info(dev, "%s():   sync_single_for_device(%p). (PCI #%d)\n", __func__, ops->sync_single_for_device, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->sync_sg_for_cpu) {
        dev_info(dev, "%s():   sync_sg_for_cpu(%p). (PCI #%d)\n", __func__, ops->sync_sg_for_cpu, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->sync_sg_for_device) {
        dev_info(dev, "%s():   sync_sg_for_device(%p). (PCI #%d)\n", __func__, ops->sync_sg_for_device, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->cache_sync) {
        dev_info(dev, "%s():   cache_sync(%p). (PCI #%d)\n", __func__, ops->cache_sync, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->dma_supported) {
        dev_info(dev, "%s():   dma_supported(%p). (PCI #%d)\n", __func__, ops->dma_supported, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->get_required_mask) {
        dev_info(dev, "%s():   get_required_mask(%p). (PCI #%d)\n", __func__, ops->get_required_mask, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->max_mapping_size) {
        dev_info(dev, "%s():   max_mapping_size(%p). (PCI #%d)\n", __func__, ops->max_mapping_size, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;
    if (ops->get_merge_boundary) {
        dev_info(dev, "%s():   get_merge_boundary(%p). (PCI #%d)\n", __func__, ops->get_merge_boundary, __LINE__);
        implemented_cnt++;
    }
    total_cnt++;

    dev_info(dev, "%s():       %d/%d implemented. (PCI #%d)\n", __func__, implemented_cnt, total_cnt, __LINE__);
}

/**
 *
 */
static void *sk_dma_alloc_coherent(struct device *dev, size_t size,
					               dma_addr_t *dma_handle, gfp_t _flag,
					               unsigned long attrs)
{
    void  * cpu_addr  = NULL;

    dev_info(dev, "%s(): intercepted. size[0x%lX]  (PCI #%d)\n", __func__, size, __LINE__);

    if (saved_dma_mapping_ops)
    {
        cpu_addr = saved_dma_mapping_ops->alloc(dev, size, dma_handle, _flag, attrs);
        dev_info(dev, "%s(): cpu_addr(%p) dma_handle(%p)  (PCI #%d)\n", __func__, cpu_addr, dma_handle, __LINE__);
        dev_info(dev, "%s(): dma_handle(%p)  (PCI #%d)\n", __func__, dma_handle, __LINE__);
#if (SETUP_DMA_MAPPING == 1)
        if(!borrower_add_to_mapping_table(dev, cpu_addr, (void *) dma_handle, size))
        {
            /* Change dma_handle fro remove device to use. */
            genXlateDmaHandle(dev, dma_handle);
            // Set the NTB mapping.
            borrower_setup_dmaAccess(dev, (dma_addr_t) dma_handle, size);

        }
#endif
    }
    return cpu_addr;
}

/**
 *
 */
static void sk_dma_free_coherent(struct device *dev, size_t size,
					             void *cpu_addr, dma_addr_t dma_handle,
					             unsigned long attrs)
{
    if (saved_dma_mapping_ops)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->free(dev, size, cpu_addr, dma_handle, attrs);
#if (SETUP_DMA_MAPPING == 1)
        borrower_remove_from_mapping_tabke(dev, cpu_addr, (void *) dma_handle, size);
#endif
        saved_dma_mapping_ops->free(dev, size, cpu_addr, dma_handle, attrs);
    }
}

/**
 *
 */
static struct page *sk_dma_alloc_pages(struct device *dev, size_t size,
			                                 dma_addr_t *dma_handle, enum dma_data_direction dir,
			                                 gfp_t gfp)
{
    struct page *page = NULL;
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->alloc_pages)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        page = saved_dma_mapping_ops->alloc_pages(dev, size, dma_handle, dir, gfp);
    }

    return page;
}

/**
 *
 */
static void sk_dma_free_pages(struct device *dev, size_t size, struct page *page,
			          dma_addr_t dma_handle, enum dma_data_direction dir)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->free_pages)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->free_pages(dev, size, page, dma_handle, dir);
    }
}

/**
 *
 */
static int sk_dma_mmap_attrs(struct device *dev, struct vm_area_struct *vma,
		void *cpu_addr, dma_addr_t dma_addr, size_t size,
		unsigned long attrs)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->mmap)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        return saved_dma_mapping_ops->mmap(dev, vma, cpu_addr, dma_addr, size, attrs);
    }
    return -ENXIO;
}


/**
 *
 */
static int sk_dma_get_sgtable_attrs(struct device *dev, struct sg_table *sgt,
		void *cpu_addr, dma_addr_t dma_addr, size_t size,
		unsigned long attrs)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->get_sgtable)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        return saved_dma_mapping_ops->get_sgtable(dev, sgt, cpu_addr, dma_addr, size, attrs);
    }
    return -ENXIO;
}

/**
 *
 */
static dma_addr_t sk_dma_map_page(struct device *dev, struct page *page,
			                      unsigned long offset, size_t size,
			                      enum dma_data_direction dir, unsigned long attrs)
{
    dma_addr_t addr = 0;

    if (saved_dma_mapping_ops && saved_dma_mapping_ops->map_page)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        addr = saved_dma_mapping_ops->map_page(dev, page, offset, size, dir, attrs);
    }

    return addr;
}

/**
 *
 */
static int sk_dma_map_sg(struct device *dev, struct scatterlist *sg, int nents,
			              enum dma_data_direction dir, unsigned long attrs)
{
    int ents = 0;

    if (saved_dma_mapping_ops && saved_dma_mapping_ops->map_sg)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        ents = saved_dma_mapping_ops->map_sg(dev, sg, nents, dir, attrs);
    }

    return ents;
}

/**
 *
 */
static void sk_dma_unmap_sg(struct device *dev, struct scatterlist *sg, int nents,
			enum dma_data_direction dir, unsigned long attrs)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->unmap_sg)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->unmap_sg(dev, sg, nents, dir, attrs);
    }
}

/**
 *
 */
static dma_addr_t sk_dma_map_resource(struct device *dev, phys_addr_t phys_addr,
			size_t size, enum dma_data_direction dir,
			unsigned long attrs)
{
    dma_addr_t addr = DMA_MAPPING_ERROR;
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->map_resource)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        addr = saved_dma_mapping_ops->map_resource(dev, phys_addr, size, dir, attrs);
    }

    return addr;
}

/**
 *
 */
static void sk_dma_unmap_resource(struct device *dev, dma_addr_t addr,
			size_t size, enum dma_data_direction dir,
			unsigned long attrs)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->unmap_resource)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->unmap_resource(dev, addr, size, dir, attrs);
    }
}

/**
 *
 */
static void sk_dma_sync_single_for_cpu(struct device *dev, dma_addr_t addr,
			                           size_t size, enum dma_data_direction dir)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->sync_single_for_cpu)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->sync_single_for_cpu(dev, addr, size, dir);
    }
}

/**
 *
 */
static void sk_dma_sync_single_for_device(struct device *dev,
			dma_addr_t addr, size_t size,
			enum dma_data_direction dir)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->sync_single_for_device)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->sync_single_for_device(dev, addr, size, dir);
    }
}

/**
 *
 */
static void sk_dma_sync_sg_for_cpu(struct device *dev, struct scatterlist *sg,
			                       int nelems, enum dma_data_direction dir)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->sync_sg_for_cpu)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->sync_sg_for_cpu(dev, sg, nelems, dir);
    }
}

/**
 *
 */
static void sk_dma_sync_sg_for_device(struct device *dev, struct scatterlist *sg,
			int nelems, enum dma_data_direction dir)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->sync_sg_for_device)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        saved_dma_mapping_ops->sync_sg_for_device(dev, sg, nelems, dir);
    }
}

/**
 *
 */
static unsigned long sk_dma_get_merge_boundary(struct device *dev)
{
    if (saved_dma_mapping_ops && saved_dma_mapping_ops->get_merge_boundary)
    {
        dev_info(dev, "%s(): intercepted  (PCI #%d)\n", __func__, __LINE__);
        return saved_dma_mapping_ops->get_merge_boundary(dev);
    }

    return 0;
}

/**
 *
 */
static const struct dma_map_ops vio_dma_mapping_ops = {
	.alloc                  = sk_dma_alloc_coherent,
	.free                   = sk_dma_free_coherent,
    .alloc_pages            = sk_dma_alloc_pages,
    .free_pages             = sk_dma_free_pages,
    .mmap                   = sk_dma_mmap_attrs,
    .get_sgtable            = sk_dma_get_sgtable_attrs,
    .map_page               = sk_dma_map_page,
    .map_sg                 = sk_dma_map_sg,
    .unmap_sg               = sk_dma_unmap_sg,
    .map_resource           = sk_dma_map_resource,
    .unmap_resource         = sk_dma_unmap_resource,
    .sync_single_for_cpu    = sk_dma_sync_single_for_cpu,
    .sync_single_for_device = sk_dma_sync_single_for_device,
    .sync_sg_for_cpu        = sk_dma_sync_sg_for_cpu,
    .sync_sg_for_device     = sk_dma_sync_sg_for_device,
    .get_merge_boundary     = sk_dma_get_merge_boundary,

};


// static void intercept(sk_vdev_t * vdev,struct device *dev)
static void intercept_init(struct device *dev)
{
    const struct dma_map_ops *ops = get_dma_ops(dev);

    // if (opsdev->dma_ops)

    dev_info(dev, "%s(): device(%p). (PCI #%d)\n", __func__, dev, __LINE__); // KW_DB
    dev_info(dev, "%s(): device->dma_ops(%p). (PCI #%d)\n", __func__, dev->dma_ops, __LINE__); // KW_DB
    // dev_info(dev, "%s(): device->dma_ops(%p). (PCI #%d)\n", __func__, dev->dma_ops, __LINE__); // KW_DB
    // dev_info(dev, "%s(): arch_dma_ops(%p). (PCI #%d)\n", __func__, get_arch_dma_ops(dev->bus);
    dev_info(dev, "%s(): ops(%p).    (PCI #%d)\n", __func__, ops, __LINE__); // KW_DB
    // if (dev->coherent_dma_mask)
    ///dev_info(dev, "%s(): device->coherent_dma_mask(x%llX). (PCI #%d)\n", __func__, dev->coherent_dma_mask, __LINE__); // KW_DB
    ///dev_info(dev, "%s(): ---- (PCI #%d)\n", __func__, __LINE__); // KW_DB
    // if (dma_alloc_direct(dev, ops))

    ///if (dev->dma_ops_bypass)
    ///   dev_info(dev, "%s(): dma_ops_bypass[x%X].    (PCI #%d)\n", __func__, dev->dma_ops_bypass, __LINE__); // KW_DB

    if (!saved_dma_mapping_ops && ops)
    {

        check_dma_ops_implementation(dev, ops);


        saved_dma_mapping_ops = (const struct dma_map_ops * ) ops;

        if (ops)
        {
            set_dma_ops(dev, &vio_dma_mapping_ops);
        // ops->alloc = (&vio_dma_mapping_ops)->alloc; //&sk_dma_alloc_coherent;
        // ops->alloc = sk_dma_alloc_coherent;
        }
    }
#if 0
    if(!ops)
    {
        // if (dma_alloc_direct(dev, ops))
        set_dma_ops(dev, &vio_dma_mapping_ops);
        dev_info(dev, "%s(): A device->dma_ops(%p). (PCI #%d)\n", __func__, dev->dev->dma_ops, __LINE__); // KW_DB
        dev_info(dev, "%s(): A ops(%p).    (PCI #%d)\n", __func__, get_dma_ops(dev->dev), __LINE__); // KW_DB
    }
#endif
}
#endif  /* INTERCEPT_DMA_API */
/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/


/**
 *
 */
int gx_vcfgspace_read(sk_vdev_t   * vdev,
                      void        * bus,
                      uint32_t      devfn,
                      int           where,
                      int           size,
                      uint32_t    * val)
{
    printk(KERN_INFO "%s(): devfn[%d]where[%d] size[%d]\n",
        __func__, devfn, where, size);
    return 0;
}

/**
 *
 */
int gx_vcfgspace_write(sk_vdev_t  * vdev,
                       void       * bus,
                       uint32_t     devfn,
                       int          where,
                       int          size,
                       uint32_t     val)
{
    printk(KERN_INFO "%s(): devfn[%d]where[%d] size[%d]\n",
        __func__, devfn, where, size);
    return 0;
}

/**
 *
 */
int sk_pci_init_vdev(sk_vdev_t       ** vdev,
                     struct pci_dev   * parent_pdev,
                     unsigned int       devfn,
                     unsigned int       type)
{

    (*vdev) = (sk_vdev_t *) kzalloc(sizeof(struct sk_vdev), GFP_KERNEL);

    if ((*vdev) == NULL) {
        printk(KERN_INFO "%s(): vdev is NULL \n", __func__);
        return -1;// OSIF_ERR_NOMEM;
    }
    else
        printk(KERN_INFO "%s(): (*vdev)(%p).\n", __func__, (*vdev));

    (*vdev)->devfn = devfn;
    (*vdev)->parent_bus = parent_pdev->bus;
    (*vdev)->type = type;

    return 0;
}


// KW_DB_NEW->
void pci_clip_resource_to_region(struct pci_bus *bus,
					struct resource *res,
					struct pci_bus_region *region)
{
	struct pci_bus_region r;

	pcibios_resource_to_bus(bus, &r, res);
	if (r.start < region->start)
		r.start = region->start;
	if (r.end > region->end)
		r.end = region->end;

	if (r.end < r.start)
		res->end = res->start - 1;
	else
		pcibios_bus_to_resource(bus, res, &r);
}

static struct pci_bus_region pci_32_bit = {0, 0xffffffffULL};
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
static struct pci_bus_region pci_64_bit = {0,
				(pci_bus_addr_t) 0xffffffffffffffffULL};
static struct pci_bus_region pci_high = {(pci_bus_addr_t) 0x100000000ULL,
				(pci_bus_addr_t) 0xffffffffffffffffULL};
#endif

// From kernel/resource.c:

/* constraints to be met while allocating resources */
struct resource_constraint {
	resource_size_t min, max, align;
	resource_size_t (*alignf)(void *, const struct resource *,
			resource_size_t, resource_size_t);
	void *alignf_data;
};

// From kernel/resource.c:
static resource_size_t simple_align_resource(void *data,
					     const struct resource *avail,
					     resource_size_t size,
					     resource_size_t align)
{
	return avail->start;
}

static void resource_clip(struct resource *res, resource_size_t min,
			  resource_size_t max)
{
	if (res->start < min)
		res->start = min;
	if (res->end > max)
		res->end = max;
}

void __weak arch_remove_reservations(struct resource *avail)
{
}

/* Return the conflict entry if you can't request it */
static struct resource * my__request_resource(struct resource *root, struct resource *new)
{
	resource_size_t start = new->start;
	resource_size_t end = new->end;
	struct resource *tmp, **p;

	if (end < start)
    {
        printk(KERN_INFO "%s(): Conflict: end < start (Ln%d) \n",
            __func__, __LINE__);
		return root;
    }
	if (start < root->start)
    {
        printk(KERN_INFO "%s(): Conflict: start < root->start (Ln%d) \n",
            __func__, __LINE__);
		return root;
    }
	if (end > root->end)
    {
        printk(KERN_INFO "%s(): Conflict: end > root->end (Ln%d) \n",
            __func__, __LINE__);
		return root;
    }
	p = &root->child;
	for (;;) {
		tmp = *p;
		if (!tmp || tmp->start > end) {
			new->sibling = tmp;
			*p = new;
			new->parent = root;
            printk(KERN_INFO "%s(): Returing NULL (Ln%d) \n", __func__, __LINE__);
			return NULL;
		}
		p = &tmp->sibling;
		if (tmp->end < start)
			continue;
        printk(KERN_INFO "%s(): Returing tmp (Ln%d) \n", __func__, __LINE__);
		return tmp;
	}
}

/*
 * Find empty slot in the resource tree with the given range and
 * alignment constraints
 */
static int my__find_resource(struct resource *root, struct resource *old,
                             struct resource *new,
                             resource_size_t  size,
                             struct resource_constraint *constraint)
{
	struct resource *this = root->child;
	struct resource tmp = *new, avail, alloc;

    printk(KERN_INFO "%s():    root: start[0x%llX] end[0x%llX] (Ln%d) \n",
        __func__, root->start, root->end, __LINE__);
    printk(KERN_INFO "%s():   child: start[0x%llX] end[0x%llX] (Ln%d) \n",
        __func__, this->start, this->end, __LINE__);
    // printk(KERN_INFO "%s():  old->start[0x%llX]  old->end[0x%llX] (Ln%d) \n",
        // __func__, old->start, old->end, __LINE__);
    if (this == old)
    {
        printk(KERN_INFO "%s():  root == old (Ln%d) \n",
        __func__, __LINE__);
    }

	tmp.start = root->start;
	/*
	 * Skip past an allocated resource that starts at 0, since the assignment
	 * of this->start - 1 to tmp->end below would cause an underflow.
	 */
	if (this && this->start == root->start)
    {
		tmp.start = (this == old) ? old->start : this->end + 1;
		this = this->sibling;
        printk(KERN_INFO "%s(): sibling: start[0x%llX] end[0x%llX] (Ln%d) \n",
            __func__, this->start, this->end, __LINE__);
        printk(KERN_INFO "%s(): tmp.start[0x%llX] (Ln%d) \n",
            __func__, tmp.start, __LINE__);
	}
	for(;;)
    {
		if (this)
			tmp.end = (this == old) ?  this->end : this->start - 1;
		else
			tmp.end = root->end;

		if (tmp.end < tmp.start)
        {
            printk(KERN_INFO "%s(): BAD: tmp.start [0x%llX] tmp.end[0x%llX] (Ln%d) \n",
            __func__, tmp.start , tmp.end, __LINE__);
			goto next;
        }
        else
            printk(KERN_INFO "%s(): OK: tmp.start [0x%llX] tmp.end[0x%llX] (Ln%d) \n",
                __func__, tmp.start , tmp.end, __LINE__);

// ... Can't even get here ...

        printk(KERN_INFO "%s(0): constraint min[0x%llX] max[0x%llX] (Ln%d) \n",
        __func__, constraint->min, constraint->max, __LINE__);
		resource_clip(&tmp, constraint->min, constraint->max);
        printk(KERN_INFO "%s(1): constraint min[0x%llX] max[0x%llX] (Ln%d) \n",
        __func__, constraint->min, constraint->max, __LINE__);
		arch_remove_reservations(&tmp);

		/* Check for overflow after ALIGN() */
		avail.start = ALIGN(tmp.start, constraint->align);
		avail.end = tmp.end;
		avail.flags = new->flags & ~IORESOURCE_UNSET;
        printk(KERN_INFO "%s(): avail.start[0x%llX] tmp.start)[0x%llX] (Ln%d) \n",
            __func__, avail.start, tmp.start, __LINE__);
		if (avail.start >= tmp.start) {
			alloc.flags = avail.flags;
			alloc.start = constraint->alignf(constraint->alignf_data, &avail,
					size, constraint->align);
			alloc.end = alloc.start + size - 1;

            printk(KERN_INFO "%s(): alloc.start[0x%llX] alloc.end[0x%llX] (Ln%d) \n",
                __func__, alloc.start, alloc.end, __LINE__);
			if (alloc.start <= alloc.end &&
			    resource_contains(&avail, &alloc)) {
				new->start = alloc.start;
				new->end = alloc.end;
				return 0;
			}
		}

next:		if (!this || this->end == root->end)
			break;

		if (this != old)
			tmp.start = this->end + 1;
		this = this->sibling;
        printk(KERN_INFO "%s(): sibling: start[0x%llX] end[0x%llX] (Ln%d) \n",
            __func__, this->start, this->end, __LINE__);

	}
	return -EBUSY;

}

/*
 * Find empty slot in the resource tree given range and alignment.
 */
static int my_find_resource(struct resource *root, struct resource *new,
			resource_size_t size,
			struct resource_constraint  *constraint)
{
	return  my__find_resource(root, NULL, new, size, constraint);
}


/**
 * allocate_resource - allocate empty slot in the resource tree given range & alignment.
 * 	The resource will be reallocated with a new size if it was already allocated
 * @root: root resource descriptor
 * @new: resource descriptor desired by caller
 * @size: requested resource region size
 * @min: minimum boundary to allocate
 * @max: maximum boundary to allocate
 * @align: alignment requested, in bytes
 * @alignf: alignment function, optional, called if not NULL
 * @alignf_data: arbitrary data to pass to the @alignf function
 */
int allocate_resource(struct resource *root, struct resource *new,
		      resource_size_t size, resource_size_t min,
		      resource_size_t max, resource_size_t align,
		      resource_size_t (*alignf)(void *,
						const struct resource *,
						resource_size_t,
						resource_size_t),
		      void *alignf_data)
{
	int err =- EINVAL;
	struct resource_constraint constraint;

	if (!alignf)
		alignf = simple_align_resource;

	constraint.min = min;
	constraint.max = max;
	constraint.align = align;
	constraint.alignf = alignf;
	constraint.alignf_data = alignf_data;

    printk(KERN_INFO "%s(): constraint min[0x%llX] max[0x%llX] (Ln%d) \n",
        __func__, min, max, __LINE__);
#if 0
	if ( new->parent ) {
		/* resource is already allocated, try reallocating with
		   the new constraints */
		return reallocate_resource(root, new, size, &constraint);
	}
#endif

#if 0
	write_lock(&resource_lock);
#endif

	err = my_find_resource(root, new, size, &constraint);
    printk(KERN_INFO "%s(): Returned from my_find_resource() err=%d (Ln%d) \n",
        __func__, err, __LINE__);
	if (err >= 0 && my__request_resource(root, new))
		err = -EBUSY;

#if 0
	write_unlock(&resource_lock);
#endif

	return err;
}


/**
 *
 */
static int my_pci_bus_alloc_from_region(struct pci_bus *bus, struct resource *res,
                                        resource_size_t size, resource_size_t align,
                                        resource_size_t min, unsigned long type_mask,
                                        resource_size_t (*alignf)(void *,
                                                         const struct resource *,
                                                         resource_size_t,
                                                         resource_size_t),
                                        void *alignf_data,
                                        struct pci_bus_region *region)
{
	int i, ret;
	struct resource *r, avail;
	resource_size_t max;

    int y=0;

	type_mask |= IORESOURCE_TYPE_BITS;

    printk(KERN_INFO "%s(): Look at each resource... (Ln%d) \n",
        __func__, __LINE__);
	pci_bus_for_each_resource(bus, r, i)
    {
		resource_size_t min_used = min;

        // printk(KERN_INFO "%s(): y[%d] (Ln%d) \n",
            // __func__, y,__LINE__);
        y++;

		if (!r)
			continue;

        printk(KERN_INFO "%s(): %d: r->start[0x%llX] r->end[0x%llX] (Ln%d) \n",
        __func__, y, r->start, r->end, __LINE__);

		/* type_mask must match */
		if ((res->flags ^ r->flags) & type_mask)
			continue;

		/* We cannot allocate a non-prefetching resource
		   from a pre-fetching area */
		if ((r->flags & IORESOURCE_PREFETCH) &&
		    !(res->flags & IORESOURCE_PREFETCH))
			continue;

		avail = *r;
		pci_clip_resource_to_region(bus, &avail, region);

		/*
		 * "min" is typically PCIBIOS_MIN_IO or PCIBIOS_MIN_MEM to
		 * protect badly documented motherboard resources, but if
		 * this is an already-configured bridge window, its start
		 * overrides "min".
		 */
		if (avail.start)
			min_used = avail.start;

		max = avail.end;

		/* Don't bother if available space isn't large enough */
		if (size > max - min_used + 1)
			continue;

		/* Ok, try it out.. */
		ret = allocate_resource(r, res, size, min_used, max,
					align, alignf, alignf_data);
        printk(KERN_INFO "%s(): Returned from allocate_resource() y[%d] ret[%d] (Ln%d) \n",
            __func__, y, ret, __LINE__);


		if (ret == 0)
			return 0;
	}
    printk(KERN_INFO "%s(): Leaving with -ENOMEM. (Ln%d) \n",
        __func__, __LINE__);
	return -ENOMEM;
}

/**
 *
 */
int my_pci_bus_alloc_resource(struct pci_bus *bus, struct resource *res,
		resource_size_t size, resource_size_t align,
		resource_size_t min, unsigned long type_mask,
		resource_size_t (*alignf)(void *,
					  const struct resource *,
					  resource_size_t,
					  resource_size_t),
		void *alignf_data)
{
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	int rc;

	if (res->flags & IORESOURCE_MEM_64) {
		rc = my_pci_bus_alloc_from_region(bus, res, size, align, min,
					       type_mask, alignf, alignf_data,
					       &pci_high);
		if (rc == 0)
			return 0;

		return my_pci_bus_alloc_from_region(bus, res, size, align, min,
						 type_mask, alignf, alignf_data,
						 &pci_64_bit);
	}
#endif

	return my_pci_bus_alloc_from_region(bus, res, size, align, min,
					 type_mask, alignf, alignf_data,
					 &pci_32_bit);
}

// KW_DB_NEW<-

/**
 *
 */
int sk_pci_add_vdev(sk_vdev_t            ** vdev,
                    struct pci_dev        * pdev,
                    unsigned int            devfn,
                    unsigned int            type,
                    vcfgspace_func_read_t   vcfg_read,
                    vcfgspace_func_write_t  vcfg_write,
                    struct perf_ctx       * perf)
{
    int rc;
    struct pci_dev        * new;
    struct pci_bus        * parent_bus = pdev->bus;
    sk_vdev_instance_t    * vdev_inst;
    struct perf_peer      * peer = NULL;

    printk(KERN_INFO "%s(): Parent Bus#[%d]\n", __func__, parent_bus->number);

    vdev_inst = vdev_instances[parent_bus->number];
    if (!vdev_inst) {
        printk(KERN_ALERT "%s(): vdev_inst is NULL.\n", __func__);
        return -1;//OSIF_ERR_INVAL;
    }

    if (perf)
        peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    if (!peer)
        return -1;

    new = pci_get_slot(parent_bus, devfn);
    if (new) {
        /* Device already present */
        printk(KERN_ALERT "%s(): Device already present.\n", __func__);
        // pci_dev_put(new);
    }
    else
    {
        printk(KERN_INFO "%s(): Calling sk_pci_init_vdev() \n", __func__);
        if ((rc = sk_pci_init_vdev(vdev, pdev, devfn, type)) != 0)
        {
            return -ENODEV;
        }
        printk(KERN_INFO "%s(): Returned from sk_pci_init_vdev() \n", __func__);
        (*vdev)->vcfgspace_read = vcfg_read;
        (*vdev)->vcfgspace_write = vcfg_write;
        (*vdev)->user_data = (void *) perf;


        // pdev->pci_ops.read = pci_read_vdevs;
        // pdev->pci_ops.write = pci_write_vdevs;


        // TODO: Add mutex is needed.
        vdev_inst->vdev_table[devfn] = (*vdev);
        ///printk(KERN_INFO "%s(): Here (Ln%d) \n", __func__, __LINE__);
        new = pci_scan_single_device((*vdev)->parent_bus, devfn);
        ///printk(KERN_INFO "%s(): Here, new(%p) (Ln%d) \n", __func__, new, __LINE__);


        /// KW_DB_20241024->
        // This is an experiment trying to false the BDF of new device to be same as the 
        // requister ID to be used when lender write data to borrow's mapper memory.
        ///new->bus->number = 0;
        ///new->devfn = 0xB;
        
        /// KW_DB_2024102<-
        
        // For debugging...
        if (new)
        {
            struct resource *res = new->resource;
            resource_size_t size;
            size = resource_size(res);
            printk(KERN_INFO "%s(): %s: resource start[0x%llX] end[0x%llX] size[0x%llX] (Ln%d) \n",
                        __func__, dev_name(&new->dev), res->start, res->end, size, __LINE__);
        }


        if (new)
        {
            phys_addr_t         ntb_base_addr;
            phys_addr_t         new_base_addr;
            resource_size_t     xlat_align, size_align, size_max;
            int                 widx, ret;  // TODO: Set wids somewhere.

			dev_info(&pdev->dev, "New device on "
				 "%02x:%02x.%d found.\n", parent_bus->number,
				 PCI_SLOT(devfn), PCI_FUNC(devfn));




            printk(KERN_INFO "%s(): vdev(%p) (Ln%d) \n", __func__, (*vdev), __LINE__);
            printk(KERN_INFO "%s(): new(%p) (Ln%d) \n", __func__, new, __LINE__);
            (*vdev)->pdev = new;



            // KW_DB_NEW->
            // Debug pci_bus_assign_resources()
            {
                // struct pci_bus *bus;

                // bus = new->bus;
                // if (bus->parent)
                    // printk(KERN_INFO "%s(): new's parent check : OK (Ln%d) \n", __func__, __LINE__);
                // else
                    // printk(KERN_ERR "%s(): new's parent check : FAILED  (Ln%d) \n", __func__, __LINE__);

                // if (bus->self->transparent)
                    // printk(KERN_INFO "%s(): new's transparent check : OK (Ln%d) \n", __func__, __LINE__);
                // else
                    // printk(KERN_ERR "%s(): new's transparent check : FAILED  (Ln%d) \n", __func__, __LINE__);


                // int idx;
                // struct pci_dev_resource *add_res,
                // struct resource *res = dev->resource + resno;


            }
            // KW_DB_NEW<-
            {// Debug pci_bus_assign_resources()

            // pci_bus_assign_resources((*vdev)->parent_bus); // KW_DB_NEW
                int resno = 0;
                // int ret = 0;
                struct resource *res;
                resource_size_t size;
                resource_size_t min;

                // Local device
                res = pdev->resource + resno;
                printk(KERN_INFO "%s(): %s: flags = 0x%lX (Ln%d) \n",
                __func__, dev_name(&pdev->dev), res->flags, __LINE__);
                size = resource_size(res);
                min = (res->flags & IORESOURCE_IO) ? PCIBIOS_MIN_IO : PCIBIOS_MIN_MEM;
                printk(KERN_INFO "%s(): %s: resource start[0x%llX] size[0x%llX] min[0x%llX](Ln%d) \n",
                    __func__, dev_name(&pdev->dev), res->start, size, min, __LINE__);

                if (res->parent)
                {   // resource is already allocated
                    printk(KERN_INFO "%s(): %s: res has parent (Ln%d) \n",
                        __func__, dev_name(&((*vdev)->pdev)->dev), __LINE__);
                }
                else
                {
                    printk(KERN_INFO "%s(): %s: res has no parent (Ln%d) \n",
                        __func__, dev_name(&pdev->dev), __LINE__);
                }

                // Remote device being added.
                res = (*vdev)->pdev->resource + resno;

                // align = pci_resource_alignment(((*vdev)->pdev)->dev, res);
                // align = resource_alignment(res);
                size = resource_size(res);
                min = (res->flags & IORESOURCE_IO) ? PCIBIOS_MIN_IO : PCIBIOS_MIN_MEM;
                printk(KERN_INFO "%s(): %s: resource start[0x%llX] end[0x%llX] size[0x%llX] min[0x%llX](Ln%d) \n",
                    __func__, dev_name(&((*vdev)->pdev)->dev), res->start, res->end, size, min, __LINE__);

                if (res->parent)
                {   // resource is already allocated
                    printk(KERN_INFO "%s(): %s: res has parent (Ln%d) \n",
                        __func__, dev_name(&((*vdev)->pdev)->dev), __LINE__);
                }
                else
                {
                    printk(KERN_INFO "%s(): %s: res has  no parent (Ln%d) \n",
                        __func__, dev_name(&((*vdev)->pdev)->dev), __LINE__);
                }

                #if 0
                // Just tying something. May not work...
                res->start = 0x100000;
                res->end = res->end + res->start;

                // Hmm..., try match resource flags...
                res->flags |= IORESOURCE_IRQ_LOWLEVEL;

                res->flags |= IORESOURCE_PREFETCH;
                res->flags |= IORESOURCE_MEM_64;
                #endif


              ///  res->flags |= IORESOURCE_MEM_64;

                printk(KERN_INFO "%s(): %s: flags = 0x%lX (Ln%d) \n",
                __func__, dev_name(&((*vdev)->pdev)->dev), res->flags, __LINE__);

                #if 1
                res->flags |= IORESOURCE_MEM_64;
                pci_bus_assign_resources((*vdev)->parent_bus);
                // ret = pci_assign_resource((*vdev)->pdev, resno);// KW_DB_NEW
                pci_dev_get(new);
                #else
                res->flags |= IORESOURCE_MEM_64;    // Crash if disable.
                ret = my_pci_bus_alloc_resource((*vdev)->parent_bus, res, size, 0x1000, min,
                                                IORESOURCE_PREFETCH | IORESOURCE_MEM_64,
                                                pcibios_align_resource, &((*vdev)->pdev)->dev);
                pci_dev_get(new);
                #endif
                printk(KERN_INFO "%s(): %s: Calling pci_bus_assign_resources() (Ln%d) \n",
                __func__, dev_name(&((*vdev)->pdev)->dev), __LINE__);
                // pci_bus_assign_resources((*vdev)->parent_bus);


                // printk(KERN_INFO "%s(): %s: Returned from pci_assign_resource() ret=%d (Ln%d) \n",
                // __func__, dev_name(&((*vdev)->pdev)->dev), ret, __LINE__);
            }
            #if 0
            //  More debug
            {
                int resno = 0;
                struct resource *res = (*vdev)->pdev->resource + resno;
                struct pci_bus *bus = (*vdev)->parent_bus;
                unsigned long type_mask = IORESOURCE_PREFETCH | IORESOURCE_MEM_64;
                int i, ret;
                struct resource *r, avail;
                resource_size_t size;
                resource_size_t max;
                resource_size_t min;
                struct pci_bus_region *region = &pci_64_bit;


                size = resource_size(res);
                min = (res->flags & IORESOURCE_IO) ? PCIBIOS_MIN_IO : PCIBIOS_MIN_MEM;


                pci_bus_for_each_resource(bus, r, i)
                {
                    resource_size_t min_used = min;

                    if (!r)
                        continue;

                    /* type_mask must match */
                    if ((res->flags ^ r->flags) & type_mask)
                        continue;

                    /* We cannot allocate a non-prefetching resource
                       from a pre-fetching area */
                    if ((r->flags & IORESOURCE_PREFETCH) &&
                        !(res->flags & IORESOURCE_PREFETCH))
                        continue;

                    avail = *r;
                    pci_clip_resource_to_region(bus, &avail, region);

                    /*
                     * "min" is typically PCIBIOS_MIN_IO or PCIBIOS_MIN_MEM to
                     * protect badly documented motherboard resources, but if
                     * this is an already-configured bridge window, its start
                     * overrides "min".
                     */
                    if (avail.start)
                        min_used = avail.start;

                    max = avail.end;

                    /* Don't bother if available space isn't large enough */
                    if (size > max - min_used + 1)
                        continue;

                    /* Ok, try it out.. */
                    // ret = allocate_resource(r, res, size, min_used, max,
                                // align, alignf, alignf_data);
                    printk(KERN_INFO "%s(): %s: Was going to call allocate_resource() (Ln%d) \n",
                        __func__, dev_name(&((*vdev)->pdev)->dev), __LINE__);
                    if (ret == 0)
                        return 0;
                }
            }
            #endif



            printk(KERN_INFO "%s(): Returned from pci_bus_assign_resources() (Ln%d) \n",
                __func__, __LINE__);
            // pci_bus_add_device(new);

            /* Get inbound MW parameters */
            widx = 5;
            ret = ntb_mw_get_align(perf->ntb, peer->pidx, widx,
                           &xlat_align, &size_align, &size_max);
            if (ret) {
                dev_err(&perf->ntb->dev, "Couldn't get pciCfgBuf restrictions\n");
                return ret;
            }

            printk(KERN_ALERT "%s: added vdev - bus: %u - devfn: %u\n",
                __FUNCTION__, (*vdev)->parent_bus->number, devfn);

            new_base_addr = pci_resource_start(new, 0);
            printk(KERN_INFO "%s(): vdev base address[x%llX].\n",
            __func__, new_base_addr);
            printk(KERN_INFO "%s(): vdev resource start[x%llX].\n",
            __func__, (new)->resource[(0)].start);
            printk(KERN_INFO "%s(): vdev resource   end[x%llX].\n",
            __func__, (new)->resource[(0)].end);

            /* Backup Original values. */
            (*vdev)->start_orig = (new)->resource[(0)].start ;
            (*vdev)->end_orig = (new)->resource[(0)].end ;

            /* Remap values. */
            ntb_base_addr = pci_resource_start(pdev, 2);
            // (new)->resource[(0)].start = ntb_base_addr + 0x60000;
            // (new)->resource[(0)].end = ntb_base_addr + 0x60000 + (32 * 1024) - 1;
            (new)->resource[(0)].start = ntb_base_addr + ((widx+1) * xlat_align);
            (new)->resource[(0)].end = ntb_base_addr + ((widx+1) * xlat_align) + SZ_32K - 1;


            printk(KERN_INFO "%s(): Updated vdev resource start[x%llX].\n",
            __func__, (new)->resource[(0)].start);
            printk(KERN_INFO "%s(): Updated vdev resource   end[x%llX].\n",
            __func__, (new)->resource[(0)].end);





            {
                // (*vdev)->dev = &new;

                // const struct dma_map_ops *ops = get_dma_ops(&new->dev);

                // dev_info(&new->dev, "%s(): device(%p). (PCI #%d)\n", __func__, &new->dev, __LINE__);
                // dev_info(&new->dev, "%s(): device->dma_ops(%p). (PCI #%d)\n", __func__, new->dev.dma_ops, __LINE__);
                // printk(KERN_INFO "%s(): ops(%p).    (PCI #%d)\n", __func__, ops, __LINE__);


                // intercept(*vdev, &new->dev);
                intercept_init(&new->dev);
                perf->new_pcie_dev = new;
            }
            printk(KERN_INFO "%s(): Calling pci_bus_add_device() (Ln %d)\n", __func__, __LINE__);
            pci_bus_add_device(new);
        }
        else
        {
            printk(KERN_ALERT "%s: new = NULL\n", __FUNCTION__);
            vdev_inst->vdev_table[devfn] = NULL;
            kfree((void *) (*vdev));
            return -ENODEV;  // OSIF_ERR_NODEV
        }
    }

    return 0;
}

/**
 *
 */
void _remove_vdev(sk_vdev_instance_t  * vdev_inst,
                  sk_vdev_t           * vdev,
                  unsigned int          free)
{
    uint32_t            devfn = vdev->devfn;
    struct pci_dev    * to_remove;

    if(vdev->pdev) {
        if ((to_remove = pci_get_slot(vdev->parent_bus, vdev->devfn)) == NULL)
        {
            printk(KERN_ALERT "%s: Warning: vdev->pdev has invalid state - skipping removal\n",
            __FUNCTION__);
        } else {
            pci_dev_put(vdev->pdev);

// #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
            // pci_stop_and_remove_bus_device_locked(vdev->pdev);
// #elif LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0)
            // pci_stop_and_remove_bus_device(vdev->pdev);
// #else /* LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0) */
            // pci_remove_bus_device(vdev->pdev);
// #endif
            pci_stop_and_remove_bus_device_locked(vdev->pdev);
        }
    }

    vdev->pdev = NULL;
    if (free) {
        printk(KERN_ALERT "%s: Freeing vdev(%p).\n",
            __FUNCTION__, vdev);
        kfree((void * )vdev);
        printk(KERN_ALERT "%s: vdev freed.\n",
            __FUNCTION__);
    }

    vdev_inst->vdev_table[devfn] = NULL;
}

/**
 *
 */
void sk_pci_remove_vdev(sk_vdev_t * vdev, unsigned int free)
{
    sk_vdev_instance_t    * vdev_inst;

    vdev_inst = vdev_instances[vdev->parent_bus->number];

    if (!vdev_inst)
    {
        printk(KERN_ALERT "%s: Warning: vdev_inst is NULL\n",
            __FUNCTION__);
        return;
    }

    // TODO: Add mutex if needed.
    _remove_vdev(vdev_inst, vdev, free);
}

/*=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=*/
static void __iomem *pci_map_bus(struct pci_bus *bus,
                                 unsigned int devfn,
                                 int where)
{
    int busno = bus->number;
    int slot = PCI_SLOT(devfn);
#if 0
    void __iomem *base;
#endif

    printk(KERN_INFO "pci_map_bus - Bus %d Slot %d", busno, slot);

    return 0;
}

/**
 *
 */
static int get_align_params(struct perf_peer *peer, sk_vdev_t *vdev, u32 widx)
{
	resource_size_t xlat_align, size_align, size_max;
	struct perf_ctx *perf = peer->perf;
	int ret;

    dev_info(&perf->ntb->dev, "%s(): Calling ntb_mw_get_align() widx[%d] (Ln %d)\n",
        __func__, widx, __LINE__); // KW_DB
	/* Get inbound MW parameters */
	ret = ntb_mw_get_align(perf->ntb, peer->pidx, (int) widx,
			       &xlat_align, &size_align, &size_max);
	if (ret) {
		dev_err(&perf->ntb->dev, "Couldn't get pciCfgBuf restrictions\n");
		return ret;
	}

    vdev->widx = widx;
    vdev->xlat_align = xlat_align;
    vdev->size_align = size_align;
    vdev->size_max = size_max;

    return 0;
}

/**
 *
 */

 // TODO: Implemented the following:
// static int get_free_widx()

#define PCIE_READ_VERBOSE   0
#define PCIE_WRITE_VERBOSE  1
/*
 * Special offsets (where) used for NVMe driver and NTB communication:
 *
 * 0x1000: MSI-X table          W
 * 0x2000: MSI-X PBA
 * 0x3000: MSI-X table offset
 * 0x3004: MSI-X table size
 */

/**
 *
 */
int pci_read_vdevs(struct pci_bus * bus,
                   unsigned int     devfn,
                   int              where,
                   int              size,
                   u32            * val)
{
    sk_vdev_instance_t    * vdev_inst;
    sk_vdev_t             * vdev;
    struct perf_ctx       * perf;
    struct perf_peer      * peer;
    void                  * pData;
    u32                     data = 0;
#if 0
    union ConfigRW          cmd;
#endif

    if (devfn != MY_DEVFN)
        return 0;

    vdev_inst = vdev_instances[bus->number];
    if (!vdev_inst || !vdev_inst->pci_ops_orig)
        return -EINVAL;

    vdev = vdev_inst->vdev_table[devfn];
    if(!vdev)
    {
        printk(KERN_ERR "%s(): vdev=NULL. (Ln%d) \n", __func__, __LINE__);
        return -EINVAL;
    }

    perf = vdev->user_data;
    if(!perf)
    {
        printk(KERN_ERR "%s(): perf=NULL. (Ln%d) \n", __func__, __LINE__);
        return -EINVAL;
    }

    peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    if(!peer)
    {
        printk(KERN_ERR "%s(): peer=NULL. (Ln%d) \n", __func__, __LINE__);
        return -EINVAL;
    }

    if (peer->pRemotePcieHandle)
        pData = peer->pRemotePcieHandle;
    else
    {
        printk(KERN_ERR "%s():pRemotePcieHandle = NULL.\n", __func__);
        return -1;
    }

#if 0
    cmd.fields.cmd_idx = config_rw_cmd_idx++;
    cmd.fields.flags = 0;
    cmd.fields.data_width = size;
    cmd.fields.offset = where;
    cmd.fields.data = 0x12345678;

    if (config_rw_cmd_idx >= 0xFF)
        config_rw_cmd_idx = 0;
    // borrower_cmd_send(peer, PERF_CMD_CFG_RD, cmd.qword);




    printk(KERN_ERR "%s(): configRwCmd[0x%LX]\n", __func__, cmd.qword);
#endif

    switch(size)
    {
        case 1:
        {
            u8 * pByte = (u8 *) (pData + where);
            data = (u32) *pByte;


            break;
        }
        case 2:
        {
            u16 * pWord = (u16 *) (pData + where);
            data = (u32) *pWord;
            break;
        }
        case 4:
        {
            u32 * pDWord = (u32 *) (pData + where);


            // Special handling
            switch (where)
            {
                case NTB_NVME_PEER_DNB_LO:
                    ///printk(KERN_WARNING "%s(): 0x%x: dnd lo[0x%X] \n", __func__, where, (u32) peer->dnb_addr & 0xFFFFFFFF);
                    *val = (u32) peer->dnb_addr & 0xFFFFFFFF;
                    return 0;
                case NTB_NVME_PEER_DNB_HI:
                    ///pr_warn("%s(): 0x%x: dnd hi[0x%X] \n", __func__, where, (u32) (peer->dnb_addr >> 32));
                    *val = (u32) (peer->dnb_addr >> 32);
                    return 0;
                // KW_DB_20241211->
                case NTB_NVME_PEER_MW0_LO:
                    ///printk(KERN_WARNING "%s(): 0x%x: dnd lo[0x%X] \n", __func__, where, (u32) peer->mw0_addr & 0xFFFFFFFF);
                    *val = (u32) peer->mw0_addr & 0xFFFFFFFF;
                    return 0;
                case NTB_NVME_PEER_MW0_HI:
                    ///pr_warn("%s(): 0x%x: dnd hi[0x%X] \n", __func__, where, (u32) (peer->mw0_addr >> 32));
                    *val = (u32) (peer->mw0_addr >> 32);
                    return 0;
                // KW_DB_20241211<-
                case NTB_NVME_ALIGN_GET_XLAT:
                    *val = (u32) vdev->xlat_align;
                    return 0;
                case NTB_NVME_ALIGN_GET_SIZE:
                    *val = (u32) vdev->size_align;
                    return 0;
                case NTB_NVME_ALIGN_GET_SIZE_MAX:
                    *val = (u32) vdev->size_max;
                    return 0;
            }
            // Use offset 0x2000 to trigger MSI-X PBA mapping setting.
            if ((where >= 0x2000) && (where < 0x3000))
            {
                printk(KERN_WARNING "%s(): 0x%x: Warning: MSI-X PBA being read. \n", __func__, where);
            }

# if (INTERFERE_BAR_WRITE > 0)
            switch(where)
            {
                case 0x10:
                    // KW_DB_NEW->
                    /*
                    data = (u32) *pDWord;
                    if (data == 0xFFFFFFFF)
                        data = 0xFFFF8004;
                    else
                    {
                        data &= 0xFFFFFFF0;
                        data |= 0x4;
                    }
                    */


                    //if ((data & 0xFC300000) ==  0xFC300000)
                    {

                        u64 addr = (u64) peer->dnb_addr;
                        // u32 first_3_bits;

                        if (bar_sizing)
                        {
                            data = 0xFFFFF000;
                            printk(KERN_INFO "%s(): 0x10: Skip readin \n", __func__);
                            bar_sizing = false;
                        }
                        else
                        {
                            data = (u32) *pDWord;
                            // first_3_bits = data & 0x00000007;

                            printk(KERN_INFO "%s(): 0x10: original[0x%X]\n", __func__, data);

                            addr += (5 * SZ_32K); // TODO:  Replace '5' with macro.

                            printk(KERN_INFO "%s(): 0x10: ppp[0x%llX]\n", __func__, addr);

                            // data = (addr & 0xFFFFFFFF) | first_3_bits;
                            if (bar_sizing_count <= 1)
                            {
                                *pDWord = (u32) 0;
                                printk(KERN_INFO "%s(): 0x10: Initialized to 0 \n", __func__);
                                data = 0;
                            }



                            printk(KERN_INFO "%s(): 0x10: reporting[0x%X]\n",
                                __func__, data);
                        }
                    }
                    // KW_DB_NEW<-
                    break;
                case 0x14:
                    // KW_DB_NEW->

                    {

                        u64 addr = (u64) peer->dnb_addr;
                        // u32 first_3_bits = data & 0x00000007;

                        if (bar_sizing)
                        {
                            data = 0;
                            printk(KERN_INFO "%s(): 0x10: Skip reading \n", __func__);
                            bar_sizing = false;
                        }
                        else
                        {
                            data = (u32) *pDWord;
                            printk(KERN_INFO "%s(): 0x14: original[0x%X]\n", __func__, data);

                            addr += (5 * SZ_32K); // TODO:  Replace '5' with macro.

                            printk(KERN_INFO "%s(): 0x14: ppp[0x%llX]\n", __func__, addr);

                            //data = (addr >> 32) | first_3_bits;
                            if (bar_sizing_count <= 2)
                            {
                                *pDWord = (u32) 0;
                                printk(KERN_INFO "%s(): 0x14: Initialized to 0 \n", __func__);
                            }
                            else
                                data = (u32) *pDWord;
                            printk(KERN_INFO "%s(): 0x14: reporting[0x%X]\n",
                                __func__, data);
                        }
                    }
                    break;
                    // KW_DB_NEW<-
                case 0x18:
                case 0x1C:
                case 0x20:
                case 0x24:
                    // KW_DB_NEW->
                    if (bar_sizing)
                    {
                        data = 0;//0xFFFFFFFF;
                        printk(KERN_INFO "%s(): 0x%X: Skip reading. \n", __func__, where);
                        bar_sizing = false;
                    }
                    else
                    // KW_DB_NEW<-
                        data = (u32) *pDWord;
                    break;
                case 0x30:
                // {

                    // data = 0;
                    // printk(KERN_INFO "%s(): 0x%X: Skip reading. \n", __func__, where);
                // }
                // break;
                    // data = 0xFFFFFFFF;
                    // break;
                default:
                    data = (u32) *pDWord;
            }
            // printk(KERN_INFO "\t\t\tRd pDWord(%p)\n", pDWord + where);
            break;
#else
            data = (u32) *pDWord;
            break;
#endif
        }
        default:
        {
            printk(KERN_ALERT "%s(): Invalid data width[%d]\n",
            __func__, size);
            return 0;
        }
    }

    *val = data;
#if (PCIE_READ_VERBOSE)
    printk(KERN_INFO "%s(): devfn[%d] where[0x%X] size[%d] data[0x%X]\n",
        __func__, devfn, where, size, *val);
#endif

    return 0;
}

/**
 * Clear reverse mappings.
 */
static void clear_reverse_mapppings(struct perf_ctx *perf)
{
    struct perf_peer  * peer;
    int                 i, y;

    peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    ///printk(KERN_ERR "%s(): lut_alloc_bit_map[0x%lX] (Ln %d) . \n", __func__, peer->lut_alloc_bit_map, __LINE__);
    
    #if 1   /* New mechanism */
    // for (y=0; y< 4; y++)                         // KW_DB_20240611
    for (y=0; y<LUT_BIT_MAP_NUM_ENTRY; y++)         // KW_DB_20240611
    {
        // for (i=0; i< SZ_64; i++)                 // KW_DB_20240611
        for (i=0; i<LUT_BIT_MAP_ENTRY_SIZE; i++)    // KW_DB_20240611
        {
            if (peer->lut_alloc_bit_map[y] & BIT(i))
            {
                printk(KERN_ERR "%s(): Calling ntb_mw_clear_trans() for widx[%d] (Ln %d) . \n", __func__, i+(y*32), __LINE__);
                // (void) ntb_mw_clear_trans(peer->perf->ntb, peer->pidx, i+(y*32));                    // KW_DB_20240611
                (void) ntb_mw_clear_trans(peer->perf->ntb, peer->pidx, i+(y*LUT_BIT_MAP_ENTRY_SIZE));   // KW_DB_20240611
                 __clear_bit(i, &peer->lut_alloc_bit_map[y]);   // KW_DB_20240611
            }
        }
    }
    #else
    for (i=0; i< SZ_64; i++)
    {
        if (peer->lut_alloc_bit_map & BIT(i))
        {
            printk(KERN_ERR "%s(): Calling ntb_mw_clear_trans() for widx[%d] (Ln %d) . \n", __func__, i, __LINE__);
            (void) ntb_mw_clear_trans(peer->perf->ntb, peer->pidx, i);
        }
    }
    #endif
}
/**
 * Perform reverse mapping so injected device can access local memory space.
 */
int do_reverse_mapping(struct perf_ctx * perf, int widx, dma_addr_t addr, resource_size_t size)
{
    resource_size_t     xlat_align, size_align, size_max;
    int                 ret = 0;
    struct perf_peer  * peer;

    /*
     *  1. Use ntb_mw_set_trans() to program incoming translation.
     *  2. Program MSI-X table entry with new a) Message address
     *                                        b) Message upper address
     */
    peer = &perf->peers[0];   // TODO: We should have only 1 port for now.

	/* Get inbound MW parameters */
	ret = ntb_mw_get_align(perf->ntb, peer->pidx, widx,
			       &xlat_align, &size_align, &size_max);

	if (ret) {
		printk(KERN_ERR "reverse_mapping restrictions\n");
		return ret;
	}

	if (size > size_max) {
		printk(KERN_ERR "size too big: size %pa > max %pa\n",
			&size, &size_max);
		return -EINVAL;
	}

    size = round_up(size, size_align);

    printk(KERN_INFO "%s(): Map LUT %d to [0x%llX] size[0x%llX]. \n", __func__, widx, addr, size);
    ret = ntb_mw_set_trans(perf->ntb, peer->pidx, widx, addr, size);

    if (ret)
        printk(KERN_ERR "%s(): Error(%d)! . \n", __func__, ret);
    else
    {
        #if 1   /* New mechanism */
        int entry = widx / LUT_BIT_MAP_ENTRY_SIZE;

        // __set_bit(widx, &peer->lut_alloc_bit_map[entry]);                                // KW_DB_20240611
        __set_bit(widx-(entry*LUT_BIT_MAP_ENTRY_SIZE), &peer->lut_alloc_bit_map[entry]);    // KW_DB_20240611 
        printk(KERN_INFO "%s(): lut_alloc_bit_map[%d] [0x%lX] (Ln %d) . \n", __func__, entry, peer->lut_alloc_bit_map[entry], __LINE__);
        #else
        // peer->lut_alloc_bit_map |= BIT(widx);
        __set_bit(widx, &peer->lut_alloc_bit_map);
        printk(KERN_INFO "%s(): lut_alloc_bit_map[0x%lX] (Ln %d) . \n", __func__, peer->lut_alloc_bit_map, __LINE__);
        #endif
    }

    return ret;
}

/**
 *
 */
static u32 my_round_down(u32 addr)
{
    u32 new_val;
    u32 lut_xlat_align = 0x8000;    // TODO: Make dynamic.
    
    new_val = round_down(addr, lut_xlat_align);
    
    printk("%s() orig_val[0x%X] new_val[0x%X] (Ln %d)\n",
        __func__, addr, new_val, __LINE__); // KW_DB
    
    return new_val;
}
    
/**
 *
 */
int pci_write_vdevs(struct pci_bus    * bus,
                    unsigned int        devfn,
                    int                 where,
                    int                 size,
                    u32                 _val)
{
#if 1
    sk_vdev_instance_t    * vdev_inst;
    sk_vdev_t             * vdev;
    struct perf_ctx       * perf;
    struct perf_peer      * peer;
    void                  * pData;
    u32                     orig_data = 0;
    u32                     new_data = 0;
    u32                     val = _val;
    union ConfigRW          cmd;

    if (devfn != MY_DEVFN)
        return 0;

    vdev_inst = vdev_instances[bus->number];
    if (!vdev_inst || !vdev_inst->pci_ops_orig)
        return -EINVAL;

    vdev = vdev_inst->vdev_table[devfn];
    if(!vdev)
    {
        printk(KERN_ERR "%s(): vdev=NULL. (Ln%d) \n", __func__, __LINE__);
        return -EINVAL;
    }

    perf = vdev->user_data;
    if(!perf)
    {
        printk(KERN_ERR "%s(): perf=NULL. (Ln%d) \n", __func__, __LINE__);
        return -EINVAL;
    }

    peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    if(!peer)
    {
        printk(KERN_ERR "%s(): peer=NULL. (Ln%d) \n", __func__, __LINE__);
        return -EINVAL;
    }

    if (peer->pRemotePcieHandle)
        pData = peer->pRemotePcieHandle;
    else
    {
        printk(KERN_ERR "%s():pRemotePcieHandle = NULL.\n", __func__);
        return -1;
    }

#if (PCIE_WRITE_VERBOSE)
    printk(KERN_INFO "%s(): devfn[%d] where[0x%X] size[%d] val[0x%X]\n",
        __func__, devfn, where, size, val);
#endif

    cmd.fields.cmd_idx = config_rw_cmd_idx++;
    cmd.fields.flags = 0;
    cmd.fields.data_width = size;
    cmd.fields.offset = where;
    cmd.fields.data = (u32) val;


    if (config_rw_cmd_idx >= 0xFF)
        config_rw_cmd_idx = 0;


    switch(size)
    {
        case 1:
        {
            u8 * pByte = (u8 *) (pData + where);
            orig_data = (u32) *pByte;
            /* Try to ask lender to update pciCfgBug and write to device instead. */
            // *pByte = (u8) val;
            borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);
            break;
        }
        case 2:
        {
            u16 * pWord = (u16 *) (pData + where);
            orig_data = (u32) *pWord;
            /* Try to ask lender to update pciCfgBug and write to device instead. */
            // *pWord = (u16) val;

    	    // KW_DB_20250723->
            // Tries to keep PCIe Max_Payload_Size at 128 bytes MPS
            if (where == 0x78)
            {
                u16 origPayloadEnc = cmd.fields.data & 0x00E0;
                origPayloadEnc = origPayloadEnc >> 5;
                cmd.fields.data &= 0xFF1F;
                //_printk(KERN_INFO "%s(): Orig MaxPayloadEnc=0x%X , new_val=0x%X. (Ln %d) \>
            }
            // KW_DB_20250723<-

            borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);
            break;
        }
        case 4:
        {
            u32   * pDWord = (u32 *) (pData + where);
            u32     msix_table_offset = NTB_NVME_MSIX_TBL_ENTRY_LO; // Offset used for NVMe & NTB communication only
            u32     msix_table_size = 0x1000;   // TODO: Fix this. However 0x1000 is min mapping size accepted.

            orig_data = (u32) *pDWord;



            // Use offset 0x1000 to trigger MSI-X table mapping setting.
            if ((where >= msix_table_offset) && (where < (msix_table_offset + msix_table_size)))
            {
                u32 offset; // Real offset
                u32 entry;  // Slot number

                printk(KERN_INFO "%s(): 0x%x: Skip writing val[0x%X]. \n", __func__, where, val);

                offset = where - msix_table_offset;
                entry = offset / 16;    // 16-bytes per entry

                printk(KERN_INFO "%s(): 0x%x: offset[0x%X] entry[0x%X]. \n", __func__, where, offset, entry);

                if ((offset & 0x0000000F) == 0)   // Set address lo
                {
#if 1   /* Start of new mechanism */

                    // vdev->msix_widx[entry] = val & ADDR_LO_WIDX_MASK;
                    // vdev->msix_widx[entry] >>= ADDR_LO_WIDX_SHIFT;
                    vdev->msix_addr_lo[entry] = val & ADDR_LO_ADDR_MASK;
                    printk(KERN_INFO "%s(): 0x%x: msix_addr_lo[0x%X]. (Ln %d)\n", __func__, entry, vdev->msix_addr_lo[entry], __LINE__ );    // KW_DB_20240815
#else                    
                    vdev->msix_addr_lo[entry] = val;
#endif                
                }
                else if ((offset & 0x0000000F) == 4)    // Set address hi
                {
                    u64 addr;
#if 1   /* Start of new mechanism */

                    vdev->msix_widx[entry] = val & ADDR_HI_WIDX_MASK;
                    vdev->msix_widx[entry] >>= ADDR_HI_WIDX_SHIFT;
                    addr = vdev->msix_addr_hi[entry] = val & ADDR_HI_ADDR_MASK;
                    printk(KERN_INFO "%s(): 0x%x: addr[0x%llX]. (Ln %d)\n", __func__, entry, addr, __LINE__ );    // KW_DB_20240815
                    addr <<=32;
                    
                    printk(KERN_INFO "%s(): 0x%x: msix_addr_lo[0x%X]. (Ln %d)\n", __func__, entry, vdev->msix_addr_lo[entry], __LINE__ );    // KW_DB_20240815
                    addr |= vdev->msix_addr_lo[entry];
                    printk(KERN_INFO "%s(): 0x%x: addr[0x%llX]. (Ln %d)\n", __func__, entry, addr, __LINE__ );    // KW_DB_20240815
                    do_reverse_mapping(perf, vdev->msix_widx[entry], addr, msix_table_size); 
#else
                    vdev->msix_addr_hi[entry] = val;
                    addr = val;
                    addr <<=32;
                    addr |= vdev->msix_addr_lo[entry];
                    do_reverse_mapping(perf, 6, addr, msix_table_size);                   
#endif                
                }
                else
                    printk(KERN_ERR "%s(): 0x%x: Invalid location. \n", __func__, where);

                return 0;
            }




# if (INTERFERE_BAR_WRITE > 0)
            switch(where)
            {
                // KW_DB_NEW->
                case 0x10:
                case 0x14:
                case 0x18:
                case 0x1C:
                case 0x20:
                case 0x24:
                    {
                        /* Local host want to read bar size */
                        if (val == 0xFFFFFFFF)
                        {
                            cmd.fields.flags |= CFG_RW_FLAGS_BUF_ONLY;
                            bar_sizing = true;
                            bar_sizing_count++;
                            printk(KERN_INFO "%s(): 0x%x: Skip writing. \n", __func__, where);
                            return 0;
                        }
                    }
                    break;
                case 0x30:
                {
                    printk(KERN_INFO "%s(): 0x%x: Skip writing. \n", __func__, where);
                    return 0;
                }
                
                case 0x54:  // MSI Capability Message Address
                {
                    if (val)
                    {
                        u32 initial_addr;   // MSI adddress
                        u32 realign_addr_lo;
                        u32 realign_offset;
                                            
                        initial_addr = val;
                        realign_addr_lo = my_round_down(initial_addr);
                        realign_offset = ((u32)(initial_addr & 0xFFFFFFFF)) - realign_addr_lo;
                        
                        
                        vdev->msi_addr_lo = realign_addr_lo;
                        
                        vdev->msi_redirect_addr = peer->dnb_addr + ((NVME_MSI_WIDX+1) * SZ_32K)  + realign_offset;
                        cmd.fields.data = (u32) (vdev->msi_redirect_addr & 0xFFFFFFFF);
                        borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);
                        return 0;
                    }
                    else
                    {
                        vdev->msi_addr_lo = val;
                        cmd.fields.data = val;
                        borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);
                        return 0;
                    }
                }
                case 0x58:  // MSI Capability Message Upper Address
                {
                    if (vdev->msi_addr_lo)
                    {
                        u64 map_addr;

                        cmd.fields.data = (u32) (vdev->msi_redirect_addr >> 32);
                        borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);
                        
                        map_addr = vdev->msi_addr_hi = val;
                        map_addr <<= 32;
                        map_addr |= vdev->msi_addr_lo;
                        do_reverse_mapping(perf, NVME_MSI_WIDX, map_addr, 0x1000);                    
                        return 0;
                    }
                    else
                    {
                        vdev->msi_addr_lo = val;
                        cmd.fields.data = val;
                        borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);
                        return 0;
                    }
                }
                // break;

                // case 0x1000:    // MSI-X table
                // {
                    // printk(KERN_INFO "%s(): 0x%x: Skip writing val[0x%X]. \n", __func__, where, val);
                    // return 0;
                // }

                case NTB_NVME_MSIX_TBL_OFFSET:
                {
                    printk(KERN_INFO "%s(): 0x%x: Set MSI-X Table offsetl[0x%X]. \n", __func__, where, val);
                    vdev->msix_table_offset = val;
                    return 0;
                }
                case NTB_NVME_MSIX_TBL_SIZE:
                {
                    printk(KERN_INFO "%s(): 0x%x: Set MSI-X Table size[0x%X]. \n", __func__, where, val);
                    vdev->msix_table_size = val;
                    return 0;
                }
#if 0   /* Start of old mechanism */
                case NTB_NVME_PRP1_ADDR_LO:
                {
                    vdev->prp1_addr_lo[0] = val;
                    return 0;
                }
                case NTB_NVME_PRP1_ADDR_HI:
                {
                    u64 addr;

                    vdev->prp1_addr_hi[0] = val;
                    addr = val;
                    addr <<=32;
                    addr |= vdev->prp1_addr_lo[0];
                    do_reverse_mapping(perf, 9, addr, 0x1000);
                    return 0;
                }
                case NTB_NVME_ASQ_ADDR_LO:
                {
                    vdev->sq_addr_lo[0] = val;
                    return 0;
                }
                case NTB_NVME_ASQ_ADDR_HI :
                {
                    u64 addr;

                    vdev->sq_addr_hi[0] = val;
                    addr = val;
                    addr <<=32;
                    addr |= vdev->sq_addr_lo[0];
                    do_reverse_mapping(perf, 7, addr, 0x1000);
                    return 0;
                }
                case NTB_NVME_ACQ_ADDR_LO:
                {
                    vdev->cq_addr_lo[0] = val;
                    return 0;
                }
                case NTB_NVME_ACQ_ADDR_HI:
                {
                    u64 addr;

                    vdev->cq_addr_hi[0] = val;
                    addr = val;
                    addr <<=32;
                    addr |= vdev->cq_addr_lo[0];
                    do_reverse_mapping(perf, 8, addr, 0x1000);
                    return 0;
                }
#else   /* Start of new mechanism */

                case NTB_NVME_PRP1_ADDR_LO:
                {
                    u32 qid;

                    qid = val & ADDR_LO_QID_MASK;
                    // vdev->prp1_widx[qid] = val & ADDR_LO_WIDX_MASK;
                    // vdev->prp1_widx[qid] >>= ADDR_LO_WIDX_SHIFT;
                    vdev->prp1_addr_lo[0] = val & ADDR_LO_ADDR_MASK;
                    
                    return 0;
                }
                case NTB_NVME_PRP1_ADDR_HI:
                {
                    u64 addr;
                    u32 qid;

                    qid = val & ADDR_HI_QID_MASK;
                    qid >>= ADDR_HI_QID_SHIFT;
                    vdev->prp1_widx[qid] = val & ADDR_HI_WIDX_MASK;
                    vdev->prp1_widx[qid] >>= ADDR_HI_WIDX_SHIFT;
                    addr = vdev->prp1_addr_hi[0] = val & ADDR_HI_ADDR_MASK;
                    addr <<= 32;
                    addr |= vdev->prp1_addr_lo[0];
                    do_reverse_mapping(perf, vdev->prp1_widx[qid] , addr, 0x1000);
                    return 0;
                }

                case NTB_NVME_ASQ_ADDR_LO:
                case NTB_NVME_SQ_ADDR_LO:
                {
                    u32 qid;

                    qid = val & ADDR_LO_QID_MASK;
                    // vdev->sq_widx[qid] = val & ADDR_LO_WIDX_MASK;
                    // vdev->sq_widx[qid] >>= ADDR_LO_WIDX_SHIFT;
                    vdev->sq_addr_lo[qid] = val & ADDR_LO_ADDR_MASK;
                    printk(KERN_INFO "%s(): qid[%d] widx[%d] sq_addr_lo[0x%X]\n",
                        __func__, qid, vdev->sq_widx[qid] , vdev->sq_addr_lo[qid]);

                    return 0;
                }
                case NTB_NVME_ASQ_ADDR_HI :
                case NTB_NVME_SQ_ADDR_HI:
                {
                    u64 addr;
                    u32 qid;

                    qid = val & ADDR_HI_QID_MASK;
                    qid >>= ADDR_HI_QID_SHIFT;
                    vdev->sq_widx[qid] = val & ADDR_HI_WIDX_MASK;
                    vdev->sq_widx[qid] >>= ADDR_HI_WIDX_SHIFT;
                    addr = vdev->sq_addr_hi[qid] = val & ADDR_HI_ADDR_MASK;
                    addr <<= 32;
                    addr |= vdev->sq_addr_lo[qid];
                    printk(KERN_INFO "%s(): qid[%d] widx[%d] sq_addr_hi[0x%X]\n",
                        __func__, qid, vdev->sq_widx[qid] , vdev->sq_addr_hi[qid]);                    
                    do_reverse_mapping(perf, vdev->sq_widx[qid], addr, 0x1000);

                    return 0;
                }
                case NTB_NVME_ACQ_ADDR_LO:
                case NTB_NVME_CQ_ADDR_LO:
                {
                    u32 qid;

                    qid = val & ADDR_LO_QID_MASK;
                    // vdev->cq_widx[qid] = val & ADDR_LO_WIDX_MASK;
                    // vdev->cq_widx[qid] >>= ADDR_LO_WIDX_SHIFT;
                    vdev->cq_addr_lo[qid] = val & ADDR_LO_ADDR_MASK;
                    printk(KERN_INFO "%s(): qid[%d] widx[%d] cq_addr_lo[0x%X]\n",
                        __func__, qid, vdev->cq_widx[qid] , vdev->cq_addr_lo[qid]);                    

                    return 0;
                }
                case NTB_NVME_ACQ_ADDR_HI:
                case NTB_NVME_CQ_ADDR_HI:
                {
                    u64 addr;
                    u32 qid;

                    qid = val & ADDR_HI_QID_MASK;
                    qid >>= ADDR_HI_QID_SHIFT;
                    vdev->cq_widx[qid] = val & ADDR_HI_WIDX_MASK;
                    vdev->cq_widx[qid] >>= ADDR_HI_WIDX_SHIFT;
                    addr = vdev->cq_addr_hi[qid] = val & ADDR_HI_ADDR_MASK;
                    addr <<= 32;
                    addr |= vdev->cq_addr_lo[qid];
                    printk(KERN_INFO "%s(): qid[%d] widx[%d] cq_addr_hi[0x%X]\n",
                        __func__, qid, vdev->cq_widx[qid] , vdev->cq_addr_hi[qid]);                     
                    do_reverse_mapping(perf, vdev->cq_widx[qid], addr, 0x1000);

                    return 0;
                }
#endif  /* End of new mechanism */            

                case NTB_NVME_ALIGN_SET_WIDX:
                {
                    get_align_params(peer, vdev, val);
                    return 0;
                }

                default:
                    if (!(cmd.fields.flags & CFG_RW_FLAGS_BUF_ONLY))
                        borrower_cmd_send(peer, PERF_CMD_CFG_WR, cmd.qword);

                    // if (val == 0xFFFFFFFF)
                        // val = 0;
                    // break;
            // KW_DB_NEW<-
            }
#else
            // if (where == 0x10)  // KW_DB_NEW
            // {
                // val = 0xABCD0000;
                // printk(KERN_INFO "%s(): 0x%x: Write 0x%X instead. \n", __func__, where, val);

            // }
#endif
            /* Try to ask lender to update pciCfgBug and write to device instead. */
            // *pDWord = (u32) val;



            new_data = (u32) *pDWord;
            // printk(KERN_INFO "\t\t\tNew val [0x%x]\n", (u32) *pDWord );
            // printk(KERN_INFO "\t\t\tWr pDWord(%p)\n", pDWord + where);
            break;
        }
        default:
        {
            printk(KERN_ALERT "%s(): Invalid data width[%d]\n",
            __func__, size);
            return 0;
        }
    }


    #if 0 // KW_DB - Probably not needed anymore
    if (size == 4)
    {
        u32 * pDWord = (u32 *) (pData + where);
        new_data = (u32) *pDWord;
        if (val == new_data)
            printk(KERN_INFO "\t\t\tData updated from [0x%x] to [0x%x]\n",
                orig_data, val);
        else
            printk(KERN_INFO "\t\t\tData failed to be updated from [0x%x] to [0x%x]\n",
                orig_data, val);
    }
    else
    {
        printk(KERN_INFO "\t\t\tData updating from [0x%x] to [0x%x]\n",
        orig_data, val);
    }
    #endif
#endif
    return 0;
}

static struct pci_ops pci_ops_vdevs = {
    .map_bus = pci_map_bus,
    .read = pci_read_vdevs,
    .write = pci_write_vdevs,
};

/**
 *
 */
int register_vdevs(struct perf_ctx *perf)
{
    struct pci_dev        * pdev = perf->ntb->pdev;
    struct pci_bus        * parent_bus;
    sk_vdev_instance_t    * vdev_inst;
    struct perf_peer      * peer;
    phys_addr_t             base_addr;
    resource_size_t         xlat_align, size_align, size_max;
    int                     widx, ret;  // TODO: Set wids somewhere.

    if (perf)
        peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    else
        return -1;
    parent_bus = pdev->bus;
    if (!parent_bus) {
        printk(KERN_ALERT "parent_bus failed!\n");
    }


    printk(KERN_INFO "%s(): lut_alloc_bit_map[] entry size[0x%lX] (Ln %d) . \n", __func__, sizeof(peer->lut_alloc_bit_map[0]), __LINE__);

    vdev_inst = (sk_vdev_instance_t *) kmalloc(sizeof(struct sk_vdev_instance), GFP_KERNEL);
    if (!vdev_inst)
        printk(KERN_ALERT "vdev_inst allocation failed!\n");
    else
        printk(KERN_INFO "vdev_inst(%p) created.\n", vdev_inst);


    base_addr = pci_resource_start(pdev, 2);
    if (!base_addr)
        return -1;

    widx = 4; // TODO:  Replace '4' with macro.
    /* Get inbound MW parameters */
    ret = ntb_mw_get_align(perf->ntb, peer->pidx, widx,
                   &xlat_align, &size_align, &size_max);
    if (ret) {
        dev_err(&perf->ntb->dev, "Couldn't get pciCfgBuf restrictions\n");
        return ret;
    }

    // peer->pRemotePcieHandle = devm_ioremap_uc(&perf->ntb->dev, base_addr + 0x50000, 64 *1024);
    peer->pRemotePcieHandle = devm_ioremap_uc(&perf->ntb->dev, base_addr + ((widx+1) * xlat_align), SZ_32K);
    if (!peer->pRemotePcieHandle)
        return -1;

    widx++;
    /* Get inbound MW parameters */
    ret = ntb_mw_get_align(perf->ntb, peer->pidx, widx,
                   &xlat_align, &size_align, &size_max);
    if (ret) {
        dev_err(&perf->ntb->dev, "Couldn't get pciCfgBuf restrictions\n");
        return ret;
    }
    // peer->pRemoteBarHandle = devm_ioremap_uc(&perf->ntb->dev, base_addr + 0x60000, 32 *1024);
    peer->pRemoteBarHandle = devm_ioremap_uc(&perf->ntb->dev, base_addr + ((widx+1) * xlat_align), SZ_32K);
    if (!peer->pRemoteBarHandle)
    {
        devm_iounmap(&perf->ntb->dev, peer->pRemotePcieHandle);
        return -1;
    }

    {   // Test reading data from mapped locations.
        u32 data;

        data = (u32) * (u32 *) peer->pRemotePcieHandle;
        printk(KERN_INFO "%s(): PCIe data[x%X].\n", __func__, data);
        data = (u32) * (u32 *) peer->pRemoteBarHandle;
        printk(KERN_INFO "%s(): Bar  data[x%X].\n", __func__, data);
    }


    vdev_inst->pci_ops_orig = pci_bus_set_ops(parent_bus, &pci_ops_vdevs);
    // printk(KERN_INFO "%s(): vdev_inst(%p).\n", __func__, vdev_inst);
    // peer->pci_ops.read = pci_read_vdevs;
    // peer->pci_ops.write = pci_write_vdevs;


    vdev_instances[parent_bus->number] = vdev_inst;

    return 0;
}

/**
 *
 */
void unregister_vdevs(struct perf_ctx *perf)
{
    struct pci_dev        * pdev = perf->ntb->pdev;
    struct pci_bus        * parent_bus;
    sk_vdev_instance_t    * vdev_inst;
    struct perf_peer      * peer;
    int                     i;

    if (perf)
        peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    else
        return;

    vdev_inst = vdev_instances[pdev->bus->number];

    if (!vdev_inst)
    {
        printk(KERN_ALERT "vdev_inst NULL!\n");
        return;
    }

    for (i = 0; i < 256; ++i) {
        if(vdev_inst->vdev_table[i]) {
            sk_pci_remove_vdev(vdev_inst->vdev_table[i], 1);
        }
    }

    parent_bus = pdev->bus;
    pci_bus_set_ops(parent_bus, (struct pci_ops *)vdev_inst->pci_ops_orig);

    printk(KERN_INFO "Freeing vdev_inst(%p).\n", vdev_inst);
    kfree((void*) vdev_inst);
    vdev_instances[pdev->bus->number] = NULL;

    if (peer->pRemotePcieHandle)
    {
        devm_iounmap(&perf->ntb->dev, peer->pRemotePcieHandle);
        peer->pRemotePcieHandle = NULL;
    }
    if (peer->pRemoteBarHandle)
    {
        devm_iounmap(&perf->ntb->dev, peer->pRemoteBarHandle);
        peer->pRemoteBarHandle = NULL;
    }
}


/**
 *
 */
static int attach_device(struct perf_ctx *perf)
{
    struct pci_dev *pdev = perf->ntb->pdev;
    struct pci_dev *newDev;
    int             rc = 0;

    printk(KERN_INFO "pdev->devfn[x%X]\n",pdev->devfn);
    printk(KERN_INFO "Slot[x%X]\n",PCI_SLOT(pdev->devfn));
    printk(KERN_INFO "Func[x%X]\n",PCI_FUNC(pdev->devfn));
    printk(KERN_INFO "BUS[x%X]\n", PCI_BUS_NUM(pdev->devfn));
    printk(KERN_INFO "BUS[x%X]\n", pdev->bus->number);
    printk(KERN_INFO "DEVFN[x%X]\n", PCI_DEVFN(0, 4));
    dev_info(&pdev->dev, "%s(): Possible CPU cnt[%d] (Ln %d)\n", __func__, num_possible_cpus(), __LINE__);

    /* Checking... */
    if (MAX_NVME_Q_PAIRS < (num_possible_cpus()*2 + 1))
    {
        printk(KERN_ERR "Error: Resource constraints.\n");
    }

    // KW_DB->
    {
        struct pci_dev *dev;
        int devfn = PCI_DEVFN(0, 4);
        u32 l;

        dev = pci_get_slot(pdev->bus, PCI_DEVFN(0, 4));
        if (dev)
        {
            printk(KERN_INFO "pci_get_slot() passed.\n");
        }
        else
        {
            printk(KERN_ERR "pci_get_slot() failed.\n");
        }


        // dev = pci_scan_device(pdev->bus, PCI_DEVFN(0, 4));
        // if (dev)
        // {
            // printk(KERN_ERR "pci_scan_device() passed.\n");
        // }
        // else
        // {
            // printk(KERN_ERR "pci_scan_device() failed.\n");
        // }

        pci_bus_read_config_dword(pdev->bus, devfn, PCI_VENDOR_ID, &l);
        printk(KERN_ERR "%s(): l = 0x%X.\n",__func__, l);

    }
    // KW_DB<-


    /*
     * 0000:00:14.0 => domain (16 bits)
     *                    bus ( 8 bits)
     *                 device ( 5 bits)
     *               function ( 3 bits)
     */
    // newDev = pci_scan_single_device(pdev->bus, PCI_DEVFN(0, 3));
    newDev = pci_scan_single_device(pdev->bus, PCI_DEVFN(0, 4));

    if (!newDev)
        printk(KERN_ERR "newDev not found\n");
    else
        printk(KERN_INFO "newDev found\n");

    printk(KERN_INFO "%s(): Calling register_vdevs() (Ln %d)\n", __func__, __LINE__);
    register_vdevs(perf);
    // rc = sk_pci_add_vdev(&sk_vdev, pdev, MY_DEVFN, 1, gx_vcfgspace_read, gx_vcfgspace_write, NULL);
    // printk(KERN_INFO "%s(): perf(%p).\n", __func__, perf);

    // For debugging...
    if (newDev)
    {
        struct resource *res = newDev->resource;
        resource_size_t size;
        size = resource_size(res);
        printk(KERN_INFO "%s(): %s: resource start[0x%llX] end[0x%llX] size[0x%llX] (Ln%d) \n",
                    __func__, dev_name(&newDev->dev), res->start, res->end, size, __LINE__);
    }

    // KW_DB_20241211->
    #if (USE_PRP_MW)
    {
        struct perf_peer      * peer = &perf->peers[0];
        int                     ret;
        // As of now, a 4GB MW is assigned for PRP mapping.
        ret = ntb_mw_set_trans(perf->ntb, peer->pidx, 1, 0, 0x100000000);
        if (ret == 0)
            printk(KERN_INFO "%s(): MW 1 programmed for PRP Mappings. (Ln %d)\n", __func__, __LINE__);
        else
            printk(KERN_ERR "%s(): Failed to program MW 1 (ret=%d) (Ln %d)\n", __func__, ret, __LINE__);
            
    }
    #endif
    // KW_DB_20241211<-
    
    printk(KERN_INFO "%s(): Calling sk_pci_add_vdev() (Ln %d)\n", __func__, __LINE__);
    rc = sk_pci_add_vdev(&sk_vdev, pdev, MY_DEVFN, 1, gx_vcfgspace_read, gx_vcfgspace_write, perf);


    return rc;
}

/**
 *
 */
int borrower_connect_remote_device(u64 bar)
{
    return 0;
}

/**
 *
 */
static ssize_t proc_borrow_write(struct file *file, const char __user *buf,
                                size_t count, loff_t *offp)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,00)  // KW_DB
    struct perf_ctx *perf =  PDE_DATA(file_inode(file));
// KW_DB->
#else
    struct perf_ctx    *perf =  pde_data(file_inode(file));
#endif
// KW_DB<-
    int ret, num_tokens;
    unsigned domain, bus, device, function; //, devfn;
    // const char *namep;
    char tmp[128] = {0};
    char name[128] = {0};
    // struct pci_dev *pdev = NULL;
	// u16 pci_data;
    // u32 dword = 0;
    // u64 qword = 0UL;
	// int rc;

    ret = copy_from_user(tmp, buf, min_t(size_t, 128, count));
    if (ret) {
        return ret;
    }

    num_tokens = sscanf(tmp, "%x:%x:%x.%x %127c", &domain, &bus, &device, &function, name);

        // if (num_tokens < 4) {
        // return -EINVAL;
    // }

    printk("%s(): tmp[%s] (Ln %d)\n", __func__, tmp, __LINE__);

    attach_device(perf);


    return count;
}



static struct proc_dir_entry   *proc_borrow_root;
static struct proc_dir_entry   *proc_borrow_devices;
static struct proc_dir_entry   *proc_borrow;


static const struct proc_ops proc_borrow_ops = {
    .proc_write = proc_borrow_write,
};

/**
 *
 */
int borrow_user_init(struct perf_ctx *perf, struct proc_dir_entry *proc_skio_root)
{
    proc_borrow_root = proc_mkdir("remote", proc_skio_root );
    if(!proc_borrow_root) {
        return -1;
    }

    proc_borrow_devices = proc_mkdir("devices", proc_borrow_root );
    if(!proc_borrow_devices) {
        proc_remove(proc_borrow_root);
        return -1;
    }

    // proc_scan = proc_create("scan", S_IWUSR, proc_borrow_root, &proc_scan_ops);

    pr_info("%s(): Here (Ln %d)\n", __func__, __LINE__);
    proc_borrow = proc_create_data("borrow", S_IWUSR, proc_skio_root, &proc_borrow_ops, perf);
    pr_info("%s(): Here (Ln %d)\n", __func__, __LINE__);

    return 0;
}

/**
 *
 */
int borrow_user_exit(struct perf_ctx *perf) {
    // proc_remove(proc_scan);
    
    if (perf->new_pcie_dev) // KW_DB_20250328
        pci_disable_device(perf->new_pcie_dev); // KW_DB_20240627
    
pr_info("%s(): Entered (Ln %d)\n", __func__, __LINE__);
unregister_vdevs(perf);

#if 0    // proc_remove() should take care of them.
    remove_proc_entry("borrow", perf->proc_skio_root);
    remove_proc_entry("devices",proc_borrow_root);
    remove_proc_entry("remote", perf->proc_skio_root);
#endif

    clear_reverse_mapppings(perf);  // KW_DB
    /// pr_info("%s(): Here (Ln %d)\n", __func__, __LINE__);
    ///pci_stop_and_remove_bus_device(perf->new_pcie_dev);  // KW_DB
    
    ///pci_proc_detach_device(perf->new_pcie_dev); // KW_DB 
    ///pci_stop_dev(perf->new_pcie_dev); // KW_DB 
    ///pci_stop_bus_device(perf->new_pcie_dev); // KW_DB 
    ///pci_proc_detach_device(perf->new_pcie_dev); // KW_DB 
    
    // pr_info("%s(): Here (Ln %d)\n", __func__, __LINE__);
    // pci_release_regions(perf->new_pcie_dev); // KW_DB
    // pr_info("%s(): Here (Ln %d)\n", __func__, __LINE__);
    // pci_disable_device(perf->new_pcie_dev); // KW_DB
    // pr_info("%s(): Here (Ln %d)\n", __func__, __LINE__);
    // pci_dev_put(perf->new_pcie_dev); // KW_DB
    
    
    // proc_remove(proc_borrow_devices);
    // proc_remove(proc_borrow_root);

    pr_info("%s(): Leaving (Ln %d)\n", __func__, __LINE__); // KW_DB
    /// remove_proc_subtree("devices", NULL);  // // KW_DB
    /// remove_proc_subtree("remote", NULL);  // KW_DB
    return 0;
}
