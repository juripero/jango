
#include <linux/module.h>   /* Needed by all modules */
#include <linux/kernel.h>   /* Needed for KERN_INFO */
#include <linux/sysfs.h>
#include <linux/pci.h>
#include <linux/ntb.h>

/* LINUX_VERSION_CODE */
#include <linux/version.h>

/* procfs */
#include <linux/proc_fs.h>

#include "ntb_device_lending.h"

MODULE_LICENSE("Dual BSD/GPL");

/**
 *
 */
static void lender_free_pciCfgBuf(struct perf_peer *peer, int widx)
{
	if (!peer->pciCfgBuf)
		return;

    dev_info(&peer->perf->ntb->dev, "%s(): pciCfgBuf DW0: 0x%X (Ln %d)\n",
        __func__, *((u32 *) peer->pciCfgBuf), __LINE__); // KW_DB
        
	(void)ntb_mw_clear_trans(peer->perf->ntb, peer->pidx, widx);
	dma_free_coherent(&peer->perf->ntb->pdev->dev, peer->pciCfgBuf_size,
			  peer->pciCfgBuf, peer->pciCfgBuf_xlat);
	peer->pciCfgBuf = NULL;
}

/**
 *
 */
static int lender_setup_pciCfgBuf(struct perf_peer *peer, int widx)
{
	resource_size_t xlat_align, size_align, size_max;
	struct perf_ctx *perf = peer->perf;
	int ret;
       
    dev_info(&perf->ntb->dev, "%s(): Calling ntb_mw_get_align() (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	/* Get inbound MW parameters */
	ret = ntb_mw_get_align(perf->ntb, peer->pidx, widx,
			       &xlat_align, &size_align, &size_max);
	if (ret) {
		dev_err(&perf->ntb->dev, "Couldn't get pciCfgBuf restrictions\n");
		return ret;
	}
    
    peer->pciCfgBuf_size = PCI_CFG_SIZE;  // TODO: Revise.

	if (peer->pciCfgBuf_size > size_max) {
		dev_err(&perf->ntb->dev, "Too big pciCfgBuf size %pa > %pa\n",
			&peer->pciCfgBuf_size, &size_max);
		return -EINVAL;
	}

    dev_info(&perf->ntb->dev, "%s(): xlat_align[x%llX] size_align[x%llX] size_max[x%llX] (Ln %d)\n",
        __func__, (u64) xlat_align, (u64) size_align, (u64) size_max, __LINE__); // KW_DB
        
	peer->pciCfgBuf_size = round_up(peer->pciCfgBuf_size, size_align);

    dev_info(&perf->ntb->dev, "%s(): pciCfgBuf_size [x%llX] (Ln %d)\n",
        __func__, (u64) peer->pciCfgBuf_size, __LINE__); // KW_DB
        
	lender_free_pciCfgBuf(peer, widx);

	peer->pciCfgBuf = dma_alloc_coherent(&perf->ntb->pdev->dev,
					 peer->pciCfgBuf_size, &peer->pciCfgBuf_xlat,
					 GFP_KERNEL);
                     
    dev_info(&perf->ntb->dev, "%s(): pciCfgBuf(%p) pciCfgBuf_size[%lld] (Ln %d)\n",
        __func__, peer->pciCfgBuf, peer->pciCfgBuf_size, __LINE__); // KW_DB
        
    dev_info(&perf->ntb->dev, "%s(): phy may be [0x%llX] (Ln %d)\n",
        __func__, virt_to_phys(peer->pciCfgBuf), __LINE__); // KW_DB


        
    ///dev_info(&perf->ntb->dev, "%s(): pAddr[0x%x] (Ln %d)\n",
    ///    __func__, dma_to_phys(&perf->ntb->pdev->dev, peer->pciCfgBuf), __LINE__); // KW_DB
        
	if (!peer->pciCfgBuf) {
		dev_err(&perf->ntb->dev, "Failed to alloc pciCfgBuf of %pa\n",
			&peer->pciCfgBuf_size);
		return -ENOMEM;
	}
	if (!IS_ALIGNED(peer->pciCfgBuf_xlat, xlat_align)) {
		dev_err(&perf->ntb->dev, "Unaligned pciCfgBuf allocated\n");
		goto err_free_pciCfgBuf;
	}

    dev_info(&perf->ntb->dev, "%s(): pciCfgBuf_xlat[x%llX] pciCfgBuf_size[%lld] (Ln %d)\n",
        __func__, (u64) peer->pciCfgBuf_xlat, peer->pciCfgBuf_size, __LINE__); // KW_DB
	// ret = ntb_mw_set_trans(perf->ntb, peer->pidx, peer->gidx,
			        // peer->pciCfgBuf_xlat, peer->pciCfgBuf_size); // KW_DB
	ret = ntb_mw_set_trans(perf->ntb, peer->pidx, widx, 
			       peer->pciCfgBuf_xlat, peer->pciCfgBuf_size); // KW_DB
    ///*(u32*)(peer->pciCfgBuf) = 0x20201010;  // KW_DB
    
	if (ret) {
		dev_err(&perf->ntb->dev, "Failed to set pciCfgBuf translation\n");
		goto err_free_pciCfgBuf;
	}
    memset(peer->pciCfgBuf, 0x00, peer->pciCfgBuf_size); // KW_DB

	/*
	 * We submit pciCfgBuf xlat transmission cmd for execution here to follow
	 * the code architecture, even though this method is called from service
	 * work itself so the command will be executed right after it returns.
	 */
    #if 0   // Disable for now.
            // TODO: Enabe if needed
    printk("%s(): exec PERF_CMD_SXLAT (Ln %d)\n",
        __func__, __LINE__); // KW_DB
	(void)perf_cmd_exec(peer, PERF_CMD_SXLAT);
    #endif
	return 0;

err_free_pciCfgBuf:
	lender_free_pciCfgBuf(peer, widx);

	return ret;
}


/**
 *
 */
static void lender_remove_barAccess(struct perf_peer *peer, int widx)
{
	(void)ntb_mw_clear_trans(peer->perf->ntb, peer->pidx, widx);
}

/**
 *
 */
static int lender_setup_barAccess(struct perf_peer *peer, int widx, u64 bar)
{
    struct perf_ctx    *perf = peer->perf;
    int                 ret = 0;

	ret = ntb_mw_set_trans(perf->ntb, peer->pidx, widx, 
			        (dma_addr_t) bar, 0x1000); // KW_DB

	if (ret) {
		dev_err(&perf->ntb->dev, "Failed to hook up BAR to LUT entry.\n");
	}
// err:
    return ret;
}

/* Connect to borrow side */
static ssize_t proc_connect_write(struct file *file, const char __user *buf, size_t count, loff_t *offp)
{
    return 0;
}

/* Add device to list of lendables */
static ssize_t proc_add_lendable_write(struct file *file, const char __user *buf, size_t count, loff_t *offp)
{
    return 0;
}

/* Remove device from list of lendables */
static ssize_t proc_rm_lendable_write(struct file *file, const char __user *buf,
                                       size_t count, loff_t *offp)
{
    return 0;
}

/* Remove device from list of lendables */
static ssize_t proc_start_write(struct file *file, const char __user *buf,
                                size_t count, loff_t *offp)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,17,00)  // KW_DB
    struct perf_ctx    *perf =  PDE_DATA(file_inode(file));
// KW_DB->
#else
    struct perf_ctx    *perf =  pde_data(file_inode(file));
#endif
// KW_DB<-
    int                 ret, num_tokens;
    unsigned            domain, bus, device, function, devfn;
    const char         *namep;
    char                tmp[128] = {0};
    char                name[128] = {0};
    struct pci_dev     *pdev = NULL;
	u16                 pci_data;
    u32                 dword = 0;
    u64                 qword = 0UL;
	int                 rc;
    
    ret = copy_from_user(tmp, buf, min_t(size_t, 128, count));
    if (ret)
        return ret;
    
    num_tokens = sscanf(tmp, "%x:%x:%x.%x %127c", &domain, &bus, &device, &function, name);
    
        if (num_tokens < 4) {
        return -EINVAL;
    }
    
    devfn = PCI_DEVFN(device, function);

    /* Find and get a reference to the device. Note that this reference is not
     * dropped here, but we keep it until lending_remove_device is called
     */
    pdev = pci_get_domain_bus_and_slot(domain, bus, devfn);
    if (pdev == NULL) {
        printk("%s(): ENODEV (Ln %d)\n", __func__, __LINE__);
        return -ENODEV;
    }

    if (num_tokens == 5) {
        namep = name;
        printk("%s(): %s (Ln %d)\n", __func__, namep, __LINE__);
    } else {
        namep = pci_name(pdev);
        printk("%s(): %s (Ln %d)\n", __func__, namep, __LINE__);
    }

	{   /* Test read config registers. */
        rc = pci_read_config_word(pdev, PCI_VENDOR_ID,
                          &pci_data);
        printk("%s():     VENDOR_ID[0x%X] (Ln %d)\n", __func__, pci_data, __LINE__);
        rc = pci_read_config_word(pdev, PCI_DEVICE_ID,
                          &pci_data);
        printk("%s(): PCI_DEVICE_ID[0x%X] (Ln %d)\n", __func__, pci_data, __LINE__);
                   
        rc = pci_read_config_dword(pdev, 0,
                          &dword);          
        ///printk("%s(): Offset 0: [0x%X] (ntb_user_Lender.c #%d)\n", __func__, dword, __LINE__);
    }

    {   /* Read config space into buffer */
        int i =0;
        struct perf_peer *peer;
       
        peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
        rc = lender_setup_pciCfgBuf(peer, 4);
        if (rc)
            return rc;
        
        /* Read config space into buffer. 4-bytes at a time. */
        while (i<PCI_CFG_SIZE)
        {
            pci_read_config_dword(pdev, i, peer->pciCfgBuf+i);
            // pr_info("%s(): pciCfgBuf offset %d: [x%X] \n", 
                // __func__, i, *(u32*) (peer->pciCfgBuf+i));
            i+=4;
        }
    
    
        /* Read and tell peer about the BAR */
        rc = pci_read_config_dword(pdev, PCI_BASE_ADDRESS_1, &dword);
        printk("%s(): PCI_BASE_ADDRESS_1[0x%X] (Ln %d)\n",
            __func__, dword, __LINE__);  
        qword = (u64) dword;
        qword <<= 32;
        rc = pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &dword);
        printk("%s(): PCI_BASE_ADDRESS_0[0x%X] (Ln %d)\n",
            __func__, dword, __LINE__);
        dword &= 0xFFFFFFF0;
        qword |= (u64) dword;

        printk("%s(): Base address[0x%llX] (Ln %d)\n",
            __func__, qword, __LINE__);
    
        /* Map device BAR for borrower to acess.*/
        lender_setup_barAccess(peer, 5, qword);
        lender_cmd_send(peer, PERF_CMD_DEV_REG, qword);            
    }
    return count;
}

extern struct proc_dir_entry    *proc_skio_root;
static struct proc_dir_entry    *proc_connect;
static struct proc_dir_entry    *proc_add_lendable;
static struct proc_dir_entry    *proc_rm_lendable;

static struct proc_dir_entry    *proc_start;

static const struct proc_ops proc_connect_ops = {
    .proc_write = proc_connect_write,
};

static const struct proc_ops proc_add_lendable_ops = {
    .proc_write = proc_add_lendable_write,
};

static const struct proc_ops proc_rm_lendable_ops = {
    .proc_write = proc_rm_lendable_write,
};

static const struct proc_ops proc_start_ops = {
    .proc_write = proc_start_write,
};




/**
 *
 */
int lending_user_init(struct perf_ctx       *perf, 
                      struct proc_dir_entry *proc_skio_root)
{
    pr_info("%s(): Here\n", __func__);
    
    proc_connect = proc_create("connect", S_IWUSR, proc_skio_root, &proc_connect_ops);
    proc_add_lendable = proc_create("add_lendable", S_IWUSR, proc_skio_root, &proc_add_lendable_ops);
    proc_rm_lendable = proc_create("rm_lendable", S_IWUSR, proc_skio_root, &proc_rm_lendable_ops);

    // proc_start = proc_create("start", S_IWUSR, proc_skio_root, &proc_start_ops);
    proc_start = proc_create_data("start", S_IWUSR, proc_skio_root, &proc_start_ops, perf);

    return 0;
}





/**
 *
 */
int lending_user_exit(struct perf_ctx *perf)
{
    struct perf_peer *peer;
    pr_info("%s(): Entered (Ln %d)\n", __func__, __LINE__);
    peer = &perf->peers[0];   // TODO: We should have only 1 port for now.
    lender_free_pciCfgBuf(peer, 4);
    lender_remove_barAccess(peer, 4);
#if 0    // proc_remove() should take care of them.
    proc_remove(proc_connect);
    proc_remove(proc_add_lendable);
    proc_remove(proc_rm_lendable);
#endif
    pr_info("%s(): Leaving (Ln %d)\n", __func__, __LINE__); // KW_DB

    return 0;
}
