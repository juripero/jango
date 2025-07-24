/*==============================================================================
 *                         Perf driver data definition
 *==============================================================================
 */

#ifndef SK_HYNIX
#define SK_HYNIX            (1) // KW_DB
#endif


#define MAX_THREADS_CNT		32
#define DEF_THREADS_CNT		1
#define MAX_CHUNK_SIZE		SZ_1M
#define MAX_CHUNK_ORDER		20 /* no larger than 1M */

#define DMA_TRIES		100
#define DMA_MDELAY		10

#define MSG_TRIES		1000
#define MSG_UDELAY_LOW		1000000
#define MSG_UDELAY_HIGH		2000000

#define PERF_BUF_LEN 1024

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
    PERF_CMD_DEV_REG = 13,  /* Lender  : Share device register address */
    PERF_CMD_DNB_ADR = 14,  /* Lender  : Device-side NTB BAR address that shared device should acess */
    // KW_DB_20241211->
    // PERF_CMD_CFG_RD  = 15,  /* Borrower: Config space read on device borrowed */
    // PERF_CMD_CFG_WR  = 16,  /* Borrower: Config space write to device borrowed */
    PERF_CMD_MW0_ADR = 15,  /* Lender  : Device-side NTB 1st MW adress */
    PERF_CMD_CFG_RD  = 16,  /* Borrower: Config space read on device borrowed */
    PERF_CMD_CFG_WR  = 17,  /* Borrower: Config space write to device borrowed */
    // KW_DB_20241211<-
#endif
};

// struct Config_rw
// {
   // union // anonymous union
   // {
       // struct { int i, j; }; // anonymous structure
       // struct { long k, l; } w;
   // };
   // int m;
// } Config_rw_t;

typedef struct Config_cmd {
    u64 data:      32; // Bits 31-00: data
    u64 offset:    16; // Bits 47-32: config space offset 
    u64 cmd_idx:    8; // Bits 55-48: cmd idx
    u64 flags:      4; // Bits 59-56: spare 
    u64 data_width: 4; // Bits 63-60: data width
} configRW_t;

union ConfigRW {
    u64         qword;
    configRW_t  fields;
};
    
#define CFG_RW_FLAGS_BUF_ONLY   (1) // Write to buffer only, not to device



struct perf_ctx;

#if (SK_HYNIX == 1)
#  define PCI_CFG_SIZE      (0x1000)
#endif

/* LUT Map to keep track of allocations */
#define LUT_BIT_MAP_NUM_ENTRY   4
#define LUT_BIT_MAP_ENTRY_SIZE  SZ_64

struct perf_peer {
	struct perf_ctx    *perf;
	int                 pidx;
	int                 gidx;

	/* Outbound MW params */
	u64                 outbuf_xlat;
	resource_size_t     outbuf_size;
	void __iomem       *outbuf;
	phys_addr_t         out_phys_addr;
	dma_addr_t          dma_dst_addr;
	/* Inbound MW params */
	dma_addr_t          inbuf_xlat;
	resource_size_t     inbuf_size;
	void		       *inbuf;        
#if (SK_HYNIX == 1)
	dma_addr_t          pciCfgBuf_xlat;
	resource_size_t     pciCfgBuf_size;
    void	           *pciCfgBuf;  // Holds PCIe Config data
    
    /* Remote device access */
    void __iomem   *pRemotePcieHandle;
    void __iomem   *pRemoteBarHandle;    
    struct pci_ops  pci_ops;
    struct pci_ops  pci_ops_orig;
    
    /* Share device register address */
    dma_addr_t          dev_reg_addr;
    /* Device side NTB BAR */
    dma_addr_t          dnb_addr;
    // KW_DB_20241211->
    /* Device side 1st NTB MW address */
    dma_addr_t          mw0_addr;
    // KW_DB_20241211<-
    u64                 spad_cmd_data;  // Data associated with PERF_CMD_CFG_RD/PERF_CMD_CFG_WR.
#endif
	/* NTB connection setup service */
	struct work_struct	service;
	unsigned long		sts;

	struct completion   init_comp;
    /* 
     * Keep track of LUT allocations
     *
     * [0]: widx 00 -  31
     * [1]: widx 32 -  63
     * [2]: widx 64 -  95
     * [3]: widx widx 96 - 127
     */
    unsigned long   lut_alloc_bit_map[LUT_BIT_MAP_NUM_ENTRY];
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

	struct dentry     * dbgfs_dir;
    dma_addr_t          dma_handle;     // KW_DB
    void              * pBuf_virt;      // KW_DB
    struct pci_dev    * new_pcie_dev;   // KW_DB
    struct proc_dir_entry * proc_skio_root;   // KW_DB
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


int lender_cmd_send(struct perf_peer *peer, enum perf_cmd cmd, u64 data);
int borrower_cmd_send(struct perf_peer *peer, enum perf_cmd cmd, u64 data);
int borrower_connect_remote_device(u64 bar);
 int lending_user_init(struct perf_ctx *perf,struct proc_dir_entry *proc_sio_root); //
 int lending_user_exit(struct perf_ctx *perf); //