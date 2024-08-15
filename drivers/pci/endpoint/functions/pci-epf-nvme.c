// SPDX-License-Identifier: GPL-2.0
/*
 * NVMe function driver for PCI Endpoint Framework
 *
 * Copyright (C) 2019 SiFive
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/pci_ids.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/nvme.h>
#include <generated/utsrelease.h>

#include "../drivers/nvme/host/nvme.h"
#include "../drivers/nvme/host/fabrics.h"
#include "../drivers/nvme/target/nvmet.h"

/*
 * Maximum number of queue pairs. Each queue pair require 3 memory mapping:
 * one fo the submission queue, one for the completion queue and one for
 * command handling. So keep this number low-ish to not run out of iATU in
 * the PCI endpoint controller.
 */
#define PCI_EPF_NVME_MAX_NR_QUEUES	16

/*
 * Default maximum queue ID, which is also the maximum number of
 * IO queue pairs.
 */
#define PCI_EPF_NVME_QID_MAX		4

/*
 * Default maximum data transfer size: limit to 128 KB to avoid
 * excessive local memory use for buffers.
 */
#define PCI_EPF_NVME_MDTS_KB		128
#define PCI_EPF_NVME_MAX_MDTS_KB	1024

/*
 * Queue state.
 */
enum pci_epf_nvme_queue_state {
	PCI_EPF_NVME_QUEUE_DEAD = 0,
	PCI_EPF_NVME_QUEUE_LIVE,
};

/*Opcodes for the KV Commands*/
enum nvme_kv_opcode {
	nvme_cmd_kv_store 		= 0x01,
	nvme_cmd_kv_retrieve		= 0x02,
	nvme_cmd_kv_delete 		= 0x10,
	nvme_cmd_kv_exist		= 0x14,
	nvme_cmd_kv_list		= 0x06,
};

enum error_codes {
	KV_SUCCESS = 0,
	KV_ERR_INVALID_VALUE_SIZE = 0x85,
	KV_ERR_INVALID_KEY_SIZE = 0X86,
	KV_ERR_INVALID_NAMESPACE_OR_FORMAT = 0x0B,
	KV_ERR_CAPACITY_EXCEEDED = 0x81,
	KV_ERR_KEY_EXISTS = 0x89,
	KV_ERR_KEY_NOT_EXIST = 0x87,
	KV_ERR_UNRECOVERED_ERROR = 0x88,
	KV_ERR_INVALID_BUFFER_SIZE = 0X89,
};

struct nvme_kv_common_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le32			cdw2[2];
	__le64			metadata;
	union nvme_data_ptr	dptr;
	struct_group(cdws,
	__le32			cdw10;
	__u8			key_len;
	__u8			rsvd[3];
	__le32			cdw12;
	__le32			cdw13;
	__le32			cdw14;
	__le32			cdw15;
	);
};

struct nvme_kv_store_command {
	__u8			opcode;
	__u8			flags;
	__u16			command_id;
	__le32			nsid;
	__le32			cdw2[2];
	__le64			metadata;
	union nvme_data_ptr	dptr;
	struct_group(cdws,
	__le32			value_size;
	__u8			key_len;
	__u8			so;
	__u8			rsvd[2];
	__le32			cdw12;
	__le32			cdw13;
	__le32			cdw14;
	__le32			cdw15;
	);
};

struct kds {
	__u16 key_length;
	char key[16];
	__u16 rsvd;
} __packed;

struct kv_readdir_data {
	struct dir_context	ctx;
	union {
		void		*private;
		char		*dirent;
	};
	char 				*kv_path;
	unsigned int		found;
	unsigned int		number_of_keys;
	unsigned int		used;
	unsigned int		dirent_count;
	unsigned int		file_attr;
	void *buffer_of_keys;
	size_t buffer_of_keys_len;
	size_t current_position;
	size_t key_len;
	struct unicode_map	*um;
};

/* 4 is for '/kv/', 16 is for the key and + 1 for the trailing 0 */
#define KV_BASE_PATH "/kv/"
#define KV_BASE_PATH_LEN 4
#define NVME_KV_MAX_KEY_LEN 16
#define NVME_KV_MAX_PRINTABLE_KEY_LEN (NVME_KV_MAX_KEY_LEN*2)
#define KV_PATH_LEN (KV_BASE_PATH_LEN+NVME_KV_MAX_PRINTABLE_KEY_LEN+1)
#define MAX_NUM_VALUE_SIZE 2147483647
#define DISK_SPACE 1024

size_t actual_size;

#define NVME_KV_STORE_REPLACE_FLAG BIT(0)
#define NVME_KV_STORE_EXCLUSIVE_FLAG BIT(1)

#define NVME_DIRECTION_TO_HOST_BIT BIT(1)
#define NVME_DIRECTION_FROM_HOST_BIT BIT(0)

/*
if (cmd.common_kv.so & NVME_KV_STORE_UPDATE_FLAG) {
	...
}
*/

/* PRP manipulation macros */
#define pci_epf_nvme_prp_addr(ctrl, prp)	((prp) & ~(ctrl)->mps_mask)
#define pci_epf_nvme_prp_ofst(ctrl, prp)	((prp) & (ctrl)->mps_mask)
#define pci_epf_nvme_prp_size(ctrl, prp)	\
	((size_t)((ctrl)->mps - pci_epf_nvme_prp_ofst(ctrl, prp)))

static struct kmem_cache *epf_nvme_cmd_cache;

struct pci_epf_nvme;

/*
 * Host PCI memory segment for admin and IO commands.
 */
struct pci_epf_nvme_segment {
	phys_addr_t	pci_addr;
	size_t		size;
};

/*
 * Queue definition and mapping for a local PCI controller.
 */
struct pci_epf_nvme_queue {
	struct pci_epf_nvme	*epf_nvme;

	enum pci_epf_nvme_queue_state state;
	int			ref;

	u16			qid;
	u16			cqid;
	u16			size;
	u16			depth;
	u16			flags;
	u16			vector;
	u16			head;
	u16			tail;
	u16			phase;
	u32			db;

	size_t			qes;

	struct pci_epc_map	map;

	struct workqueue_struct	*cmd_wq;
	struct workqueue_struct	*cqe_wq;
	struct delayed_work	work;
	spinlock_t		lock;
	struct list_head	list;
};

/*
 * Local PCI controller exposed with an endpoint function.
 */
struct pci_epf_nvme_ctrl {
	/* Backing fabrics host controller */
	struct nvme_ctrl		*ctrl;

	/* Registers of the local PCI controller */
	void				*reg;
	u64				cap;
	u32				vs;
	u32				cc;
	u32				csts;
	u32				aqa;
	u64				asq;
	u64				acq;

	size_t				adm_sqes;
	size_t				adm_cqes;
	size_t				io_sqes;
	size_t				io_cqes;

	size_t				mps_shift;
	size_t				mps;
	size_t				mps_mask;

	size_t				mdts;

	unsigned int			nr_queues;
	struct pci_epf_nvme_queue	*sq;
	struct pci_epf_nvme_queue	*cq;
};

/*
 * Descriptor for commands sent by the host. This is also used internally for
 * fabrics commands to control our fabrics target.
 */
struct pci_epf_nvme_cmd {
	struct list_head		link;
	struct pci_epf_nvme		*epf_nvme;

	int				sqid;
	int				cqid;
	unsigned int			status;
	struct nvme_ns			*ns;
	struct nvme_command 		cmd;
	struct nvme_completion		cqe;

	/* Internal buffer that we will transfer over PCI */
	size_t				buffer_size;
	void				*buffer;
	enum dma_data_direction		dma_dir;

	/*
	 * Host PCI adress segments: if nr_segs is 1, we use only "seg",
	 * otherwise, the segs array is allocated and used to store
	 * multiple segments.
	 */
	unsigned int			nr_segs;
	struct pci_epf_nvme_segment	seg;
	struct pci_epf_nvme_segment	*segs;

	struct work_struct		io_work;
};

/*
 * EPF function private data representing our NVMe subsystem.
 */
struct pci_epf_nvme {
	struct pci_epf			*epf;
	const struct pci_epc_features	*epc_features;

	void				*reg[PCI_STD_NUM_BARS];
	enum pci_barno			reg_bar;
	size_t				msix_table_offset;

	unsigned int			irq_type;
	unsigned int			nr_vectors;

	unsigned int			queue_count;

	struct pci_epf_nvme_ctrl	ctrl;
	bool				ctrl_enabled;

	__le64				*prp_list_buf;

	struct dma_chan			*dma_chan_tx;
        struct dma_chan			*dma_chan_rx;
        struct mutex			xfer_lock;

	struct delayed_work		reg_poll;

	/* Function configfs attributes */
	struct config_group		group;
	char				*ctrl_opts_buf;
	bool				dma_enable;
	unsigned int			qid_max;
	size_t				mdts_kb;
};

/*
 * Read a 32-bits BAR register (equivalent to readl()).
 */
static inline u32 pci_epf_nvme_reg_read32(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	volatile __le32 *ctrl_reg = ctrl->reg + reg;

	return le32_to_cpu(*ctrl_reg);
}

/*
 * Write a 32-bits BAR register (equivalent to readl()).
 */
static inline void pci_epf_nvme_reg_write32(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u32 val)
{
	volatile __le32 *ctrl_reg = ctrl->reg + reg;

	*ctrl_reg = cpu_to_le32(val);
}

/*
 * Read a 64-bits BAR register (equivalent to lo_hi_readq()).
 */
static inline u64 pci_epf_nvme_reg_read64(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	return (u64)pci_epf_nvme_reg_read32(ctrl, reg) |
		((u64)pci_epf_nvme_reg_read32(ctrl, reg + 4) << 32);
}

/*
 * Write a 64-bits BAR register (equivalent to lo_hi_writeq()).
 */
static inline void pci_epf_nvme_reg_write64(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u64 val)
{
	pci_epf_nvme_reg_write32(ctrl, reg, val & 0xFFFFFFFF);
	pci_epf_nvme_reg_write32(ctrl, reg + 4, (val >> 32) & 0xFFFFFFFF);
}

static inline bool pci_epf_nvme_ctrl_ready(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	if (!epf_nvme->ctrl_enabled)
		return false;
	return (ctrl->cc & NVME_CC_ENABLE) && (ctrl->csts & NVME_CSTS_RDY);
}

struct pci_epf_nvme_dma_filter {
        struct device *dev;
        u32 dma_mask;
};

static void __bin_to_hex(const char *data, int length, char *output) {
	int i;
    for (i = 0; i < length; ++i) {
        sprintf(output + (i * 2), "%02x", data[i] & 0xFF);
    }
    output[length * 2] = '\0'; 
}

static bool pci_epf_nvme_dma_filter(struct dma_chan *chan, void *arg)
{
        struct pci_epf_nvme_dma_filter *filter = arg;
        struct dma_slave_caps caps;

        memset(&caps, 0, sizeof(caps));
        dma_get_slave_caps(chan, &caps);

        return chan->device->dev == filter->dev &&
               (filter->dma_mask & caps.directions);
}

static bool pci_epf_nvme_init_dma(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf *epf = epf_nvme->epf;
	struct device *dev = &epf->dev;
	struct pci_epf_nvme_dma_filter filter;
	struct dma_chan *chan;
	dma_cap_mask_t mask;

	mutex_init(&epf_nvme->xfer_lock);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	chan = dma_request_channel(mask, pci_epf_nvme_dma_filter, &filter);
	if (!chan)
		return false;
	epf_nvme->dma_chan_rx = chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	chan = dma_request_channel(mask, pci_epf_nvme_dma_filter, &filter);
	if (!chan) {
		dma_release_channel(epf_nvme->dma_chan_rx);
		epf_nvme->dma_chan_rx = NULL;
		return false;
	}
	epf_nvme->dma_chan_tx = chan;

	dev_info(dev, "DMA RX channel %s, maximum segment size %u B\n",
		 dma_chan_name(epf_nvme->dma_chan_rx),
		 dma_get_max_seg_size(epf_nvme->dma_chan_rx->device->dev));
	dev_info(dev, "DMA TX channel %s, maximum segment size %u B\n",
		 dma_chan_name(epf_nvme->dma_chan_tx),
		 dma_get_max_seg_size(epf_nvme->dma_chan_tx->device->dev));

	return true;
}

static void pci_epf_nvme_clean_dma(struct pci_epf_nvme *epf_nvme)
{
	if (epf_nvme->dma_chan_tx) {
		dma_release_channel(epf_nvme->dma_chan_tx);
		epf_nvme->dma_chan_tx = NULL;
	}

	if (epf_nvme->dma_chan_rx) {
		dma_release_channel(epf_nvme->dma_chan_rx);
		epf_nvme->dma_chan_rx = NULL;
	}
}

static void pci_epf_nvme_dma_callback(void *param)
{
	complete(param);
}

static ssize_t pci_epf_nvme_dma_transfer(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_segment *seg,
					 enum dma_data_direction dir, void *buf)
{
	struct pci_epf *epf = epf_nvme->epf;
	struct device *dma_dev = epf->epc->dev.parent;
	struct dma_async_tx_descriptor *desc;
	DECLARE_COMPLETION_ONSTACK(complete);
	struct dma_slave_config sconf = {};
	struct device *dev = &epf->dev;
	phys_addr_t dma_addr;
	struct dma_chan *chan;
	dma_cookie_t cookie;
	int ret;

	switch (dir) {
	case DMA_FROM_DEVICE:
		chan = epf_nvme->dma_chan_rx;
		sconf.direction = DMA_DEV_TO_MEM;
		sconf.src_addr = seg->pci_addr;
		break;
	case DMA_TO_DEVICE:
		chan = epf_nvme->dma_chan_tx;
		sconf.direction = DMA_MEM_TO_DEV;
		sconf.dst_addr = seg->pci_addr;
		break;
	default:
		return -EINVAL;
	}

	ret = dmaengine_slave_config(chan, &sconf);
	if (ret) {
		dev_err(dev, "Failed to configure DMA channel\n");
		return ret;
	}

	dma_addr = dma_map_single(dma_dev, buf, seg->size, dir);
	ret = dma_mapping_error(dma_dev, dma_addr);
	if (ret) {
		dev_err(dev, "Failed to map remote memory\n");
		return ret;
	}

	desc = dmaengine_prep_slave_single(chan, dma_addr,
					   seg->size, sconf.direction,
					   DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(dev, "Failed to prepare DMA\n");
		ret = -EIO;
		goto unmap;
	}

	desc->callback = pci_epf_nvme_dma_callback;
	desc->callback_param = &complete;

	cookie = dmaengine_submit(desc);
	ret = dma_submit_error(cookie);
	if (ret) {
		dev_err(dev, "DMA submit failed %d\n", ret);
		goto unmap;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_timeout(&complete, msecs_to_jiffies(1000));
	if (!ret) {
		dev_err(dev, "DMA transfer timeout\n");
		dmaengine_terminate_sync(chan);
		ret = -ETIMEDOUT;
		goto unmap;
	}

	ret = seg->size;

unmap:
	dma_unmap_single(dma_dev, dma_addr, seg->size, dir);

	return ret;
}

static ssize_t pci_epf_nvme_mmio_transfer(struct pci_epf_nvme *epf_nvme,
					  struct pci_epf_nvme_segment *seg,
					  enum dma_data_direction dir,
					  void *buf)
{
	struct pci_epf *epf = epf_nvme->epf;
	struct pci_epc_map map;
	ssize_t map_size, ret;

	/* Map segment */
	map_size = pci_epf_mem_map(epf, seg->pci_addr, seg->size, &map);
	if (map_size < 0)
		return map_size;

	dev_dbg(&epf->dev,
		"MMIO-mmapped PCI addr 0x%llx (0x%llx) to virt addr 0x%llx, "
		"phys addr 0x%llx, size %zd (%zu) / %zu B\n",
		map.pci_addr, map.map_pci_addr, (u64)map.virt_addr,
		map.phys_addr, map_size, map.pci_size, seg->size);

	switch (dir) {
	case DMA_FROM_DEVICE:
		memcpy_fromio(buf, map.virt_addr, map.pci_size);
		ret = map_size;
		break;
	case DMA_TO_DEVICE:
		memcpy_toio(map.virt_addr, buf, map.pci_size);
		ret = map_size;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pci_epf_mem_unmap(epf, &map);

	return ret;
}

static int pci_epf_nvme_transfer(struct pci_epf_nvme *epf_nvme,
				 struct pci_epf_nvme_segment *seg,
				 enum dma_data_direction dir, void *buf)
{
	size_t size = seg->size;
	int ret;

	while (size) {
		/*
		 * Note: mmio transfers do not need serialization, but
		 * this is an easy way to prevent using too many mapped
		 * memory area which would lead to errors.
		 */
		mutex_lock(&epf_nvme->xfer_lock);
		if (!epf_nvme->dma_enable)
			ret = pci_epf_nvme_mmio_transfer(epf_nvme, seg,
							 dir, buf);
		else
			ret = pci_epf_nvme_dma_transfer(epf_nvme, seg,
							dir, buf);
		mutex_unlock(&epf_nvme->xfer_lock);

		if (ret < 0)
			return ret;

		size -= ret;
		buf += ret;
	}

	return 0;
}

static const char *pci_epf_nvme_cmd_name(struct pci_epf_nvme_cmd *epcmd)
{
	u8 opcode = epcmd->cmd.common.opcode;

	if (epcmd->sqid)
		return nvme_get_opcode_str(opcode);
	return nvme_get_admin_opcode_str(opcode);
}

static inline const char *nvme_get_kv_opcode_str(u8 opcode)
{
	switch (opcode)
	{
	case nvme_cmd_kv_store:
		return "KV Store Cmd";
	case nvme_cmd_kv_retrieve:
		return "KV Retrieve Cmd";
	case nvme_cmd_kv_delete:
		return "KV Delete Cmd";
	case nvme_cmd_kv_exist:
		return "KV Exist Cmd";
	case nvme_cmd_kv_list:
		return "KV List Cmd";
	
	default:
		return "No KV Cmd with the opcode given";
	}
}

static const char *pci_epf_nvme_kv_cmd_name(struct pci_epf_nvme_cmd *epcmd)
{
	u8 opcode = epcmd->cmd.common.opcode;

	if (epcmd->sqid)
		return nvme_get_kv_opcode_str(opcode);
	return nvme_get_admin_opcode_str(opcode);
}

static inline struct pci_epf_nvme_cmd *
pci_epf_nvme_alloc_cmd(struct pci_epf_nvme *nvme)
{
	return kmem_cache_alloc(epf_nvme_cmd_cache, GFP_KERNEL);
}

static void pci_epf_nvme_io_cmd_work(struct work_struct *work);

static void pci_epf_nvme_init_cmd(struct pci_epf_nvme *epf_nvme,
				  struct pci_epf_nvme_cmd *epcmd,
				  int sqid, int cqid)
{
	memset(epcmd, 0, sizeof(*epcmd));
	INIT_LIST_HEAD(&epcmd->link);
	INIT_WORK(&epcmd->io_work, pci_epf_nvme_io_cmd_work);
	epcmd->epf_nvme = epf_nvme;
	epcmd->sqid = sqid;
	epcmd->cqid = cqid;
	epcmd->status = NVME_SC_SUCCESS;
	epcmd->dma_dir = DMA_NONE;
}

static int pci_epf_nvme_alloc_cmd_buffer(struct pci_epf_nvme_cmd *epcmd)
{
	void *buffer;

	buffer = kmalloc(epcmd->buffer_size, GFP_KERNEL);
	if (!buffer) {
		epcmd->buffer_size = 0;
		return -ENOMEM;
	}

	if (!epcmd->sqid)
		memset(buffer, 0, epcmd->buffer_size);
	epcmd->buffer = buffer;

	return 0;
}

static int pci_epf_nvme_alloc_cmd_segs(struct pci_epf_nvme_cmd *epcmd,
				       int nr_segs)
{
	struct pci_epf_nvme_segment *segs;

	/* Single segment case: use the command embedded structure */
	if (nr_segs == 1) {
		epcmd->segs = &epcmd->seg;
		epcmd->nr_segs = 1;
		return 0;
	}

	/* More than one segment needed: allocate an array */
	segs = kcalloc(nr_segs, sizeof(struct pci_epf_nvme_segment), GFP_KERNEL);
	if (!segs)
		return -ENOMEM;

	epcmd->nr_segs = nr_segs;
	epcmd->segs = segs;

	return 0;
}

static void pci_epf_nvme_free_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	if (epcmd->ns)
		nvme_put_ns(epcmd->ns);

	if (epcmd->buffer)
		kfree(epcmd->buffer);

	if (epcmd->segs && epcmd->segs != &epcmd->seg)
		kfree(epcmd->segs);

	kmem_cache_free(epf_nvme_cmd_cache, epcmd);
}

static void pci_epf_nvme_complete_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_queue *cq;
	unsigned long flags;

	if (!pci_epf_nvme_ctrl_ready(epf_nvme)) {
		pci_epf_nvme_free_cmd(epcmd);
		return;
	}

	/*
	 * Add the command to the list of completed commands for the
	 * target cq and schedule the list processing.
	 */
	cq = &epf_nvme->ctrl.cq[epcmd->cqid];
	spin_lock_irqsave(&cq->lock, flags);
	list_add_tail(&epcmd->link, &cq->list);
	queue_delayed_work(cq->cqe_wq, &cq->work, 0);
	spin_unlock_irqrestore(&cq->lock, flags);
}

static int pci_epf_nvme_transfer_cmd_data(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_segment *seg;
	void *buf = epcmd->buffer;
	size_t size = 0;
	int i, ret;

	/* Transfer each segment of the command */
	for (i = 0; i < epcmd->nr_segs; i++) {
		seg = &epcmd->segs[i];

		if (size >= epcmd->buffer_size) {
			dev_err(&epf_nvme->epf->dev, "Invalid transfer size\n");
			goto xfer_err;
		}

		ret = pci_epf_nvme_transfer(epf_nvme, seg, epcmd->dma_dir, buf);
		if (ret)
			goto xfer_err;

		buf += seg->size;
		size += seg->size;
	}

	return 0;

xfer_err:
	epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
	return -EIO;
}

static void pci_epf_nvme_raise_irq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_queue *cq)
{
	struct pci_epf *epf = epf_nvme->epf;
	int ret;

	if (!(cq->flags & NVME_CQ_IRQ_ENABLED))
		return;

	switch (epf_nvme->irq_type) {
	case PCI_IRQ_MSIX:
	case PCI_IRQ_MSI:
		ret = pci_epf_raise_irq(epf, epf_nvme->irq_type,
					cq->vector + 1);
		if (!ret)
			return;
		/*
		 * If we got an error, it is likely because the host is using
		 * legacy IRQs (e.g. BIOS, grub), so fallthrough.
		 */
		fallthrough;
	case PCI_IRQ_LEGACY:
		ret = pci_epf_raise_irq(epf, PCI_IRQ_LEGACY, 0);
		if (!ret)
			return;
		break;
	default:
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		break;
	}

	if (ret)
		dev_err(&epf->dev, "Raise IRQ failed %d\n", ret);
}

static struct pci_epf_nvme_cmd *
pci_epf_nvme_fetch_cmd(struct pci_epf_nvme *epf_nvme,
		       struct pci_epf_nvme_queue *sq)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_cmd *epcmd;

	if (sq->state != PCI_EPF_NVME_QUEUE_LIVE)
		return NULL;

	/* Queue empty ? */
	sq->tail = pci_epf_nvme_reg_read32(ctrl, sq->db);
	if (sq->tail == sq->head)
		return NULL;

	epcmd = pci_epf_nvme_alloc_cmd(epf_nvme);
	if (!epcmd)
		return NULL;

	/* Get the NVMe command submitted by the host */
	pci_epf_nvme_init_cmd(epf_nvme, epcmd, sq->qid, sq->cqid);
	memcpy_fromio(&epcmd->cmd,
		      sq->map.virt_addr + sq->head * sq->qes,
		      sizeof(struct nvme_command));

	dev_dbg(&epf_nvme->epf->dev,
		"sq[%d]: head %d/%d, tail %d, command %s\n",
		sq->qid, (int)sq->head, (int)sq->depth, (int)sq->tail,
		pci_epf_nvme_cmd_name(epcmd));

	sq->head++;
	if (sq->head == sq->depth)
		sq->head = 0;

	return epcmd;
}

/*
 * Transfer a prp list from the host and return the number of prps.
 */
static int pci_epf_nvme_get_prp_list(struct pci_epf_nvme *epf_nvme, u64 prp,
				     size_t xfer_len)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	size_t nr_prps = (xfer_len + ctrl->mps_mask) >> ctrl->mps_shift;
	struct pci_epf_nvme_segment seg;
	int ret;

	/*
	 * Compute the number of PRPs required for the number of bytes to
	 * transfer (xfer_len). If this number overflows the memory page size
	 * with the PRP list pointer specified, only return the space available
	 * in the memory page, the last PRP in there will be a PRP list pointer
	 * to the remaining PRPs.
	 */
	seg.pci_addr = prp;
	seg.size = min(pci_epf_nvme_prp_size(ctrl, prp), nr_prps << 3);
	ret = pci_epf_nvme_transfer(epf_nvme, &seg, DMA_FROM_DEVICE,
				    epf_nvme->prp_list_buf);
	if (ret)
		return ret;

	return seg.size >> 3;
}

static int pci_epf_nvme_cmd_parse_prp_list(struct pci_epf_nvme *epf_nvme,
					   struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	__le64 *prps = epf_nvme->prp_list_buf;
	struct pci_epf_nvme_segment *seg;
	size_t size = 0, ofst, prp_size, xfer_len;
	size_t transfer_len = epcmd->buffer_size;
	int nr_segs, nr_prps = 0;
	phys_addr_t pci_addr;
	int i = 0, ret;
	u64 prp;

	/*
	 * Allocate segments for the command: this considers the worst case
	 * scenario where all prps are discontiguous, so get as many segments
	 * as we can have prps. In practice, most of the time, we will have
	 * far less segments than prps.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp1);
	if (!prp)
		goto invalid_field;

	ofst = pci_epf_nvme_prp_ofst(ctrl, prp);
	nr_segs = (transfer_len + ofst + NVME_CTRL_PAGE_SIZE - 1)
		>> NVME_CTRL_PAGE_SHIFT;

	ret = pci_epf_nvme_alloc_cmd_segs(epcmd, nr_segs);
	if (ret)
		goto internal;

	/* Set the first segment using prp1 */
	seg = &epcmd->segs[0];
	seg->pci_addr = prp;
	seg->size = pci_epf_nvme_prp_size(ctrl, prp);

	size = seg->size;
	pci_addr = prp + size;
	nr_segs = 1;

	/*
	 * Now build the pci address segments using the prp lists, starting
	 * from prp2.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp2);
	if (!prp)
		goto invalid_field;

	while (size < transfer_len) {
		xfer_len = transfer_len - size;

		if (!nr_prps) {
			/* Get the prp list */
			nr_prps = pci_epf_nvme_get_prp_list(epf_nvme, prp,
							    xfer_len);
			if (nr_prps < 0)
				goto internal;

			i = 0;
			ofst = 0;
		}

		/* Current entry */
		prp = le64_to_cpu(prps[i]);
		if (!prp)
			goto invalid_field;

		/* Did we reach the last prp entry of the list ? */
		if (xfer_len > ctrl->mps && i == nr_prps - 1) {
			/* We need more PRPs: prp is a list pointer */
			nr_prps = 0;
			continue;
		}

		/* Only the first prp is allowed to have an offset */
		if (pci_epf_nvme_prp_ofst(ctrl, prp))
			goto invalid_offset;

		if (prp != pci_addr) {
			/* Discontiguous prp: new segment */
			nr_segs++;
			if (WARN_ON_ONCE(nr_segs > epcmd->nr_segs)) {
				dev_err(&epf_nvme->epf->dev,
					"Number of required segments is bigger than %d, size : %zu\n",
					epcmd->nr_segs, size);
				goto internal;
			}

			seg++;
			seg->pci_addr = prp;
			seg->size = 0;
			pci_addr = prp;
		}

		prp_size = min_t(size_t, ctrl->mps, xfer_len);
		seg->size += prp_size;
		pci_addr += prp_size;
		size += prp_size;

		i++;
	}

	epcmd->nr_segs = nr_segs;
	ret = 0;

	if (size != transfer_len) {
		dev_err(&epf_nvme->epf->dev,
			"PRPs transfer length mismatch %zu / %zu\n",
			size, transfer_len);
		goto internal;
	}

	return 0;

internal:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return -EINVAL;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;
}

static int pci_epf_nvme_cmd_parse_prp_simple(struct pci_epf_nvme *epf_nvme,
					     struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	size_t transfer_len = epcmd->buffer_size;
	int ret, nr_segs = 1;
	u64 prp1, prp2 = 0;
	size_t prp1_size;

	/* prp1 */
	prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	prp1_size = pci_epf_nvme_prp_size(ctrl, prp1);

	/* For commands crossing a page boundary, we should have a valid prp2 */
	if (transfer_len > prp1_size) {
		prp2 = le64_to_cpu(cmd->common.dptr.prp2);
		if (!prp2)
			goto invalid_field;
		if (pci_epf_nvme_prp_ofst(ctrl, prp2))
			goto invalid_offset;
		if (prp2 != prp1 + prp1_size)
			nr_segs = 2;
	}

	/* Create segments using the prps */
	ret = pci_epf_nvme_alloc_cmd_segs(epcmd, nr_segs);
	if (ret )
		goto internal;

	epcmd->segs[0].pci_addr = prp1;
	if (nr_segs == 1) {
		epcmd->segs[0].size = transfer_len;
	} else {
		epcmd->segs[0].size = prp1_size;
		epcmd->segs[1].pci_addr = prp2;
		epcmd->segs[1].size = transfer_len - prp1_size;
	}

	return 0;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;

internal:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return ret;
}

static int pci_epf_nvme_cmd_parse_dptr(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	u64 prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	size_t ofst;
	int ret;

	if (epcmd->buffer_size > ctrl->mdts)
		goto invalid_field;

	/* We do not support SGL for now */
	if (epcmd->cmd.common.flags & NVME_CMD_SGL_ALL)
		goto invalid_field;

	/* Get pci segments for the command using its prps */
	ofst = pci_epf_nvme_prp_ofst(ctrl, prp1);
	if (ofst & 0x3)
		goto invalid_offset;

	if (epcmd->buffer_size + ofst <= NVME_CTRL_PAGE_SIZE * 2)
		ret = pci_epf_nvme_cmd_parse_prp_simple(epf_nvme, epcmd);
	else
		ret = pci_epf_nvme_cmd_parse_prp_list(epf_nvme, epcmd);
	if (ret)
		return ret;

	/* Get an internal buffer for the command */
	ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		return ret;
	}

	return 0;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;
}

static int file_doesnt_exist(const char *path, struct pci_epf_nvme_cmd *epcmd) {
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct file *filp;
	dev_info(&epf_nvme->epf->dev, "PATH: %s\n", path);
	filp = filp_open(path, O_RDONLY, 0666);
	if (IS_ERR(filp)) {
		dev_err(&epf_nvme->epf->dev, "file does NOT exist\n");
		return 1;
	}
	else {
		filp_close(filp, NULL);
		return 0;
	}
}

static void delete_filp(struct file *filp)
{
	struct inode *parent_inode = filp->f_path.dentry->d_parent->d_inode;
	inode_lock(parent_inode);
	vfs_unlink(&nop_mnt_idmap, parent_inode, filp->f_path.dentry, NULL);
	inode_unlock(parent_inode);
}

static int delete_file(const char *path)
{
	struct file *filp;
	filp = filp_open(path, O_RDWR, 0666);
	if (IS_ERR(filp))
		return PTR_ERR_OR_ZERO(filp);
	filp_close(filp, NULL);
	delete_filp(filp);
	return 0;
}

static void DumpHex(const void* data, size_t size, 
		    struct pci_epf_nvme_cmd *epcmd) {
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	char ascii[17];
	size_t i, j;
	ascii[16] = '\0';
	for (i = 0; i < size; ++i) {
		dev_info(&epf_nvme->epf->dev, "%02X ", 
			 ((unsigned char*)data)[i]);
		if (((unsigned char*)data)[i] >= ' ' && 
		    ((unsigned char*)data)[i] <= '~') {
			ascii[i % 16] = ((unsigned char*)data)[i];
		} else {
			ascii[i % 16] = '.';
		}
		if ((i+1) % 8 == 0 || i+1 == size) {
			dev_info(&epf_nvme->epf->dev, " ");
			if ((i+1) % 16 == 0) {
				dev_info(&epf_nvme->epf->dev, "|  %s \n", 
					 ascii);
			} else if (i+1 == size) {
				ascii[(i+1) % 16] = '\0';
				if ((i+1) % 16 <= 8) {
					dev_info(&epf_nvme->epf->dev, " ");
				}
				for (j = (i+1) % 16; j < 16; ++j) {
					dev_info(&epf_nvme->epf->dev, " ");
				}
				dev_info(&epf_nvme->epf->dev, "|  %s \n", 
					 ascii);
			}
		}
	}
}

static int __hex_to_bin(const char *input, int input_length, 
			char *output, int output_length
			/*struct pci_epf_nvme_cmd *epcmd*/) {
	//struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
    	// Ensure the input has an even number of characters
	if (input_length == 0) {
		/*dev_info(&epf_nvme->epf->dev,
			"Length equals 0\n");*/
		return 0;
	}else if (input_length % 2 != 0) {
		/*dev_info(&epf_nvme->epf->dev,
			"Hexadecimal string must have an even number of "
			"characters\n");*/
		return 0;
    	}
	
	if (output_length < input_length / 2) {
		/*dev_info(&epf_nvme->epf->dev,
			 "Output buffer is too small\n");*/
		return 0;
	}

	for (int i = 0; i < input_length; i += 2) {
		char byteString[3] = {input[i], input[i + 1], '\0'};
		long byteValue;
		// Pass the address of byteValue
		int ret = kstrtol(byteString, 16, &byteValue); 

		if (ret) {
			/*dev_info(&epf_nvme->epf->dev,
			 	 "Conversion error: %d\n", ret);*/
			return 0;
		}

		// Ensure the value fits within a byte
		if (byteValue < 0 || byteValue > 255) {
			/*dev_info(&epf_nvme->epf->dev,
			 	 "Hexadecimal value out of range: %ld", 
				 byteValue);*/
			return 0;
		}

		output[i / 2] = (char)byteValue;
	}

	return 1;
}

static bool __dir_print_actor(struct dir_context *ctx, const char *name, 
			      int namlen, loff_t offset, u64 ino, 
			      unsigned int d_type
			      /*struct pci_epf_nvme_cmd *epcmd*/)
{
	//struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct kv_readdir_data *buf;
	struct kds kds;

	buf = container_of(ctx, struct kv_readdir_data, ctx);
	buf->dirent_count++;
	if (!strcmp(name, buf->kv_path + 4))  {
		buf->found = 1;
	}
	if (strlen(name) == 0) {
		return 0;
	}
	int key_length = (int)strlen(name)/2;

	if (strcmp(name, ".") && strcmp(name, "..") && buf->found) {
		kds.key_length = (unsigned int)key_length;
		if (name != NULL) {

			int ret = __hex_to_bin(name, (int)strlen(name), 
					       (void*)&kds.key, key_length
					       /*epcmd*/);
			++buf->number_of_keys;
			if (ret == 0) {
				/*dev_info(&epf_nvme->epf->dev,
				 	 "ERROR doing the transformation from "
					 "hexadecimal to binary\n");*/
				return 0;
			}
		}

		/* The Key data structure should be a multiple of 4 bytes 
		(u32) so we add 3 (sizeof(u32)-1) and integer divide by 
		sizeof(u32) to round up */
		size_t kds_size = ((sizeof(kds.key_length) + kds.key_length + 
				   (sizeof(u32) - 1)) / 
				   sizeof(u32)) * sizeof(u32);

		
		if ((buf->current_position + kds_size) <= 
		    buf->buffer_of_keys_len) {
			/*dev_info(&epf_nvme->epf->dev,
				 "ACTOR: Count: %d Name: %s\n", 
				 buf->dirent_count, name);*/
			memcpy(buf->buffer_of_keys + buf->current_position, 
			       &kds, kds_size);
			buf->current_position += kds_size;
			return 1;
		} else {
			--buf->number_of_keys;
			/*dev_info(&epf_nvme->epf->dev,
				 "The key does NOT fit in the buffer");*/
			return 0;
		}
		
	}
	return 1;
}

static void pci_epf_nvme_exec_kv_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	//struct nvme_command *cmd = &epcmd->cmd;
	struct request_queue *q;
	struct nvme_kv_common_command *cmd = (struct nvme_kv_common_command *) 
					      &epcmd->cmd.common;
	int key_length = cmd->key_len;
	int ret = 0;
	u64 cdw2_0 = le32_to_cpu(cmd->cdw2[0]);
	u64 cdw2_1 = (u64)le32_to_cpu(cmd->cdw2[1]);
	u64 cdw15 = (u64)(__le32_to_cpu(cmd->cdw15));
	u64 key_lsb = ((cdw2_1 << 32) | cdw2_0);
	u64 key_msb = ((cdw15 << 32) | __le32_to_cpu(cmd->cdw14));
	loff_t file_offset = 0;
	int value_size = 0;
	int previous_value_size = 0;
	struct file *kv_file = NULL;
	char kv_path[KV_PATH_LEN];
	char *path_key_ptr = kv_path + strlen(KV_BASE_PATH);
	struct file *fp = NULL;
	char *buf, *path;
	struct kv_readdir_data readdir_data;

	size_t effective_value_size = 0;
	char buffer[PAGE_SIZE];

	if (epcmd->ns)
		q = epcmd->ns->queue;
	else
		q = epf_nvme->ctrl.ctrl->admin_q;

	/* Transfer size is value size (cdw10) */
	epcmd->buffer_size = cmd->cdw10;

	if (epcmd->buffer_size) {
		/* Setup the command buffer */
		ret = pci_epf_nvme_cmd_parse_dptr(epcmd);
		if (ret)
			return;

		/* Get data from the host if needed */
		if (epcmd->dma_dir == DMA_FROM_DEVICE) {
			ret = pci_epf_nvme_transfer_cmd_data(epcmd);
			if (ret)
				return;
		}
	}

	dev_info(&epf_nvme->epf->dev,
		 "Command %s\n", pci_epf_nvme_kv_cmd_name(epcmd));

	
	if (epcmd->cmd.common.opcode == nvme_cmd_kv_store || 
	    epcmd->cmd.common.opcode == nvme_cmd_kv_retrieve ||
	    epcmd->cmd.common.opcode == nvme_cmd_kv_exist ||
	    epcmd->cmd.common.opcode == nvme_cmd_kv_list ||
	    epcmd->cmd.common.opcode == nvme_cmd_kv_delete) {
		
		dev_info(&epf_nvme->epf->dev, "It is a KV Command");
		
		if(key_length > 16 || key_length <= 0) {
			dev_err(&epf_nvme->epf->dev,
				"ERROR: key size NOT valid\n");
			epcmd->status = KV_ERR_INVALID_KEY_SIZE;
			return;
		}
		memset(kv_path, 0, KV_PATH_LEN);
		sprintf(kv_path, KV_BASE_PATH);

		__bin_to_hex((const char*)&key_lsb, 
			     umin((size_t)key_length,sizeof(key_lsb)), 
			     path_key_ptr);
		if (key_length > sizeof(key_lsb)) {
			__bin_to_hex((const char*)&key_msb, 
				     key_length - sizeof(key_lsb), 
				     path_key_ptr + sizeof(key_lsb)*2);
		}
		dev_info(&epf_nvme->epf->dev,
		 	 "Key length: %d Key value: %s\n", key_length,
			 kv_path);
	}

	switch (cmd->opcode) {
		case nvme_cmd_kv_store:
			struct nvme_kv_store_command *store_cmd = 
					(struct nvme_kv_store_command *) 
					 &epcmd->cmd.common;
			value_size = store_cmd->value_size;
			if(value_size < 0 || value_size > MAX_NUM_VALUE_SIZE) {
				epcmd->status = KV_ERR_INVALID_VALUE_SIZE;
				break;
			}
			if (file_doesnt_exist(kv_path, epcmd)) {	//insert
				/*
				if (value_size > actual_size) {
					epcmd->status = 
						KV_ERR_CAPACITY_EXCEEDED;
					break;
				}*/

				//if bit 8 is set to 1
				if (store_cmd->so & 
				    NVME_KV_STORE_REPLACE_FLAG) {
					dev_err(&epf_nvme->epf->dev, 
						"NO kv_store insert %s because "
						"Bit 8 set to 1\n", 
						path_key_ptr);
					epcmd->status = KV_ERR_KEY_NOT_EXIST;
					break;
				}
				
				kv_file = filp_open(kv_path, O_RDWR | 
						O_CREAT, 0666);
				if (!kv_file || IS_ERR(kv_file)) {
					dev_err(&epf_nvme->epf->dev, 
						"Could not write to file %s\n", 
						kv_path);
					dev_err(&epf_nvme->epf->dev, 
						"ERROR: %ld\n", 
						PTR_ERR(kv_file));
					epcmd->status = 
						KV_ERR_UNRECOVERED_ERROR;
					break;
				} else {
					dev_info(&epf_nvme->epf->dev, 
						"Writing data to file: %s\n",
						kv_path);
					ret = kernel_write(kv_file, 
							epcmd->buffer, 
							epcmd->buffer_size, 0);
					if (ret < 0) {
						dev_err(&epf_nvme->epf->dev, 
							"Could not write KV" 
							" value to file %s\n", 
							kv_path);
						epcmd->status = 
						    KV_ERR_UNRECOVERED_ERROR;
						break;
					}
					actual_size -= store_cmd->value_size;
					dev_info(&epf_nvme->epf->dev, 
						"ACTUAL SIZE: %zu\n",
						actual_size);
				}
			}
			else {						//update
				dev_info(&epf_nvme->epf->dev, "Is an UPDATE");
				//if bit 9 is set to 1
				if(store_cmd->so & 
				   NVME_KV_STORE_EXCLUSIVE_FLAG) {
					dev_err(&epf_nvme->epf->dev, 
						"NO kv_store update %s because "
						"Bit 9 set to 1\n", 
						path_key_ptr);
					epcmd->status = KV_ERR_KEY_EXISTS;
					break;
				}

				previous_value_size = 0;
				ret = 1;
				file_offset = 0;
				kv_file = filp_open(kv_path, O_RDWR | O_CREAT, 
						    0666);
				while(ret != 0) {
					ret = kernel_read(kv_file, 
							  buffer, PAGE_SIZE, 
							  &file_offset);
					previous_value_size += ret;
				}
				actual_size += previous_value_size;
				/*if (value_size > actual_size) {
					actual_size -= previous_value_size;
					dev_info(&epf_nvme->epf->dev, 
						 "The Update could NOT be done,"
						 " file stays as before\n");
					epcmd->status = 
						KV_ERR_CAPACITY_EXCEEDED;
					break;
				}*/
				delete_file(kv_path);
				kv_file = filp_open(kv_path, O_RDWR | O_CREAT, 
						    0666);
				dev_info(&epf_nvme->epf->dev, 
					"Writing data to file: %s\n",
					kv_path);
				ret = kernel_write(kv_file, 
						   epcmd->buffer, 
						   epcmd->buffer_size, 0);
				if (ret < 0) {
					dev_err(&epf_nvme->epf->dev, 
						"Could not write all KV" 
						" value to file %s\n", 
						kv_path);
					epcmd->status = 
						KV_ERR_UNRECOVERED_ERROR;
					break;
				}
				actual_size -= store_cmd->value_size;
				dev_info(&epf_nvme->epf->dev, 
					"ACTUAL SIZE: %zu\n",
					actual_size);
			}
			epcmd->status = KV_SUCCESS;
			dev_info(&epf_nvme->epf->dev, 
				 "File written successfully\n");
			break;
		case nvme_cmd_kv_retrieve:
			if(cmd->cdw10  <= 0 || 
			   cmd->cdw10 > MAX_NUM_VALUE_SIZE) {
				epcmd->status = KV_ERR_INVALID_BUFFER_SIZE;
				break;
			}
			kv_file = filp_open(kv_path, O_RDONLY, 0666);
			if (!kv_file || IS_ERR(kv_file)) {
				dev_info(&epf_nvme->epf->dev, 
					 "File %s does not exist\n", 
					 kv_path);
				epcmd->status = KV_ERR_KEY_NOT_EXIST;
				break;
			}else {
				dev_info(&epf_nvme->epf->dev, 
					 "Reading data from file: %s\n", 
					 kv_path);
				file_offset = 0;
				ret = kernel_read(kv_file, epcmd->buffer, 
						  epcmd->buffer_size,
						  &file_offset);
				if (ret < 0) {
					dev_err(&epf_nvme->epf->dev, 
						"Could not read KV value "
						"from file %s\n", kv_path);
					epcmd->status = 
						KV_ERR_UNRECOVERED_ERROR;
					break;
				}
				effective_value_size = ret;
				while(ret) {
					ret = kernel_read(kv_file, buffer, 
							  PAGE_SIZE,
							  &file_offset);
					if (ret < 0) {
						epcmd->status = 
						      KV_ERR_UNRECOVERED_ERROR;
						break;
					}
					effective_value_size += ret;
				}
				/* Return effective value size in CQE */
				epcmd->cqe.result.u32 = effective_value_size;
			}
			epcmd->status = KV_SUCCESS;
			dev_info(&epf_nvme->epf->dev, 
				 "File read successfully\n");
			DumpHex(epcmd->buffer, epcmd->buffer_size, epcmd);
			break;
		case nvme_cmd_kv_delete:
			epcmd->dma_dir = DMA_NONE;
			if (file_doesnt_exist(kv_path, epcmd)) {
				dev_info(&epf_nvme->epf->dev, 
					"File %s does NOT exist\n", kv_path);
				epcmd->status = KV_ERR_KEY_NOT_EXIST;
				break;
			} else {
				dev_info(&epf_nvme->epf->dev, 
					"File %s does exist\n", kv_path);
				ret = 1;
				value_size = 0;
				kv_file = filp_open(kv_path, O_RDONLY, 0666);
				if (!kv_file || IS_ERR(kv_file)) {
					dev_info(&epf_nvme->epf->dev, 
						"Could NOT open file %s\n", 
						kv_path);
					epcmd->status = KV_ERR_KEY_NOT_EXIST;
					break;
				}
				while(ret != 0) {
					ret = kernel_read(kv_file, 
							  buffer, PAGE_SIZE, 
							  &file_offset);
					value_size += ret;
				}
				actual_size += key_length;
				actual_size += value_size;
				delete_file(kv_path);
				epcmd->status = KV_SUCCESS;
				dev_info(&epf_nvme->epf->dev, 
					"File deleted successfully\n");
				break;
			}

		case nvme_cmd_kv_exist:
			epcmd->dma_dir = DMA_NONE;
			if (file_doesnt_exist(kv_path, epcmd)) {
				dev_info(&epf_nvme->epf->dev, 
					"File %s does NOT exist\n", kv_path);
				epcmd->status = KV_ERR_KEY_NOT_EXIST;
				break;
			}else {
				dev_info(&epf_nvme->epf->dev, 
					"File %s does exist\n", kv_path);
				epcmd->status = KV_SUCCESS;
				break;
			}
		case nvme_cmd_kv_list:
			if(cmd->cdw10  <= 0 || 
			   cmd->cdw10 > MAX_NUM_VALUE_SIZE) {
				epcmd->status = KV_ERR_INVALID_BUFFER_SIZE;
				break;
			}
			dev_info(&epf_nvme->epf->dev,
				 "\t--- NVME KV LIST ---\n");
			fp = filp_open("/kv", O_RDONLY | O_NONBLOCK | O_CLOEXEC 
				       | O_DIRECTORY, 0);
			if (fp == NULL) {
				epcmd->status = 
					KV_ERR_INVALID_NAMESPACE_OR_FORMAT;
				dev_info(&epf_nvme->epf->dev,
				 	 "Error opening the directory\n");
				break;	
			}
			dev_info(&epf_nvme->epf->dev, 
				 "Directory successfully open\n");
			buf = __getname();
			if (!buf) {
				epcmd->status = NVME_SC_INTERNAL;
				filp_close(fp, NULL);
				dev_info(&epf_nvme->epf->dev,
				 	 "Error allocating memory\n");
				break;
			}
			memset(buf, 0, PATH_MAX);
			path = kv_path;
			readdir_data.kv_path = path;
			readdir_data.found = 0;
			readdir_data.buffer_of_keys_len = cmd->cdw10;
			readdir_data.key_len = cmd->key_len;
			readdir_data.current_position = 4;
			readdir_data.number_of_keys = 0;
			readdir_data.buffer_of_keys = kmalloc(
				readdir_data.buffer_of_keys_len, GFP_KERNEL);
			if (!readdir_data.buffer_of_keys) {
				dev_info(&epf_nvme->epf->dev,
				 	 "Error allocating memory\n");
				break;
			}
			memset(readdir_data.buffer_of_keys, 0, 
			       readdir_data.buffer_of_keys_len);
			
			if (file_doesnt_exist(kv_path, epcmd)) {
				readdir_data.found = 1;
			}
			readdir_data.ctx.actor = __dir_print_actor;
			ret = iterate_dir(fp, &readdir_data.ctx);
			dev_info(&epf_nvme->epf->dev,
				 "Number of keys: %d Found: %d Dirent_count: %d\n", 
				 readdir_data.number_of_keys, readdir_data.found, 
				 readdir_data.dirent_count);
			memcpy(readdir_data.buffer_of_keys, 
			       &readdir_data.number_of_keys, sizeof(u32));
			epcmd->buffer = readdir_data.buffer_of_keys;
			//DumpHex(epcmd->buffer, epcmd->buffer_size, epcmd);
			DumpHex(readdir_data.buffer_of_keys, 
				readdir_data.buffer_of_keys_len, epcmd);
			__putname(buf);
			filp_close(fp, NULL);
			dev_info(&epf_nvme->epf->dev,
				 "List successfully done\n");
			break;
		default:
			dev_err(&epf_nvme->epf->dev,
				"Unhandled IO command %s (0x%02x)\n",
				pci_epf_nvme_cmd_name(epcmd),
				epcmd->cmd.common.opcode);
			epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
			
	}

	/** @todo
	 * 1) Switch based on opcode
	 * 1b) if list, fill epcmd->buffer directly (kv_readdir_data.buffer_of_keys = epcmd->buffer, buffer_of_keys_len = epcmd->buffer_size)
	 * 2) do kernel_read / kernel_write if retrieve/store (into/from epcmd->buffer from/to file)
	 */

	/* If needed transfer data to host (e.g., KV retrieve or list) */
	if (epcmd->buffer_size && epcmd->dma_dir == DMA_TO_DEVICE)
		pci_epf_nvme_transfer_cmd_data(epcmd);
}

static void pci_epf_nvme_exec_cmd(struct pci_epf_nvme_cmd *epcmd,
			void (*post_exec_hook)(struct pci_epf_nvme_cmd *))
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct nvme_command *cmd = &epcmd->cmd;
	struct request_queue *q;
	int ret;

	if (epcmd->ns)
		q = epcmd->ns->queue;
	else
		q = epf_nvme->ctrl.ctrl->admin_q;

	if (epcmd->buffer_size) {
		/* Setup the command buffer */
		ret = pci_epf_nvme_cmd_parse_dptr(epcmd);
		if (ret)
			return;

		/* Get data from the host if needed */
		if (epcmd->dma_dir == DMA_FROM_DEVICE) {
			ret = pci_epf_nvme_transfer_cmd_data(epcmd);
			if (ret)
				return;
		}
	}

	/* Synchronously execute the command */
	ret = __nvme_submit_sync_cmd(q, cmd, &epcmd->cqe.result,
				     epcmd->buffer, epcmd->buffer_size,
				     NVME_QID_ANY, 0);
	if (ret < 0)
                epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	else if (ret > 0)
		epcmd->status = ret;

	if (epcmd->status != NVME_SC_SUCCESS) {
		dev_err(&epf_nvme->epf->dev,
			"QID %d: submit command %s (0x%x) failed, status 0x%0x\n",
			epcmd->sqid, pci_epf_nvme_cmd_name(epcmd),
			epcmd->cmd.common.opcode, epcmd->status);
		return;
	}

	if (post_exec_hook)
		post_exec_hook(epcmd);

	if (epcmd->buffer_size && epcmd->dma_dir == DMA_TO_DEVICE)
		pci_epf_nvme_transfer_cmd_data(epcmd);
}

static void pci_epf_nvme_io_cmd_work(struct work_struct *work)
{
	struct pci_epf_nvme_cmd *epcmd =
		container_of(work, struct pci_epf_nvme_cmd, io_work);

	if (0) /* NVM commands */
		pci_epf_nvme_exec_cmd(epcmd, NULL);
	else /* KV commands */
		pci_epf_nvme_exec_kv_cmd(epcmd);

	pci_epf_nvme_complete_cmd(epcmd);
}

static bool pci_epf_nvme_queue_response(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf *epf = epf_nvme->epf;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[epcmd->sqid];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[epcmd->cqid];
	struct nvme_completion *cqe = &epcmd->cqe;
	unsigned long flags;

	/*
	 * Do not try to complete commands if the controller is not ready
	 * anymore, e.g. after the host cleared CC.EN.
	 */
	if (!pci_epf_nvme_ctrl_ready(epf_nvme) ||
	    cq->state != PCI_EPF_NVME_QUEUE_LIVE)
		goto free_cmd;

	spin_lock_irqsave(&cq->lock, flags);

	/* Check completion queue full state */
	cq->head = pci_epf_nvme_reg_read32(ctrl, cq->db);
	if (cq->head == cq->tail + 1) {
		spin_unlock_irqrestore(&cq->lock, flags);
		return false;
	}

	/* Setup the completion entry */
	cqe->sq_id = cpu_to_le16(epcmd->sqid);
	cqe->sq_head = cpu_to_le16(sq->head);
	cqe->command_id = epcmd->cmd.common.command_id;
	cqe->status = cpu_to_le16((epcmd->status << 1) | cq->phase);

	/* Post the completion entry */
	dev_dbg(&epf->dev,
		"cq[%d]: %s status 0x%x, head %d, tail %d, phase %d\n",
		epcmd->cqid, pci_epf_nvme_cmd_name(epcmd),
		epcmd->status, cq->head, cq->tail, cq->phase);

	memcpy_toio(cq->map.virt_addr + cq->tail * cq->qes, cqe,
		    sizeof(struct nvme_completion));

	/* Advance cq tail */
	cq->tail++;
	if (cq->tail >= cq->depth) {
		cq->tail = 0;
		cq->phase ^= 1;
	}

	spin_unlock_irqrestore(&cq->lock, flags);

free_cmd:
	pci_epf_nvme_free_cmd(epcmd);

	return true;
}

static void pci_epf_nvme_cq_work(struct work_struct *work)
{
	struct pci_epf_nvme_queue *cq =
		container_of(work, struct pci_epf_nvme_queue, work.work);
	struct pci_epf_nvme_cmd *epcmd;
	bool reschedule = false;
	unsigned long flags;
	LIST_HEAD(list);
	int nr_cqe;

	spin_lock_irqsave(&cq->lock, flags);

	while(!list_empty(&cq->list)) {

		list_splice_tail_init(&cq->list, &list);
		spin_unlock_irqrestore(&cq->lock, flags);

		nr_cqe = 0;
		while (!list_empty(&list)) {
			epcmd = list_first_entry(&list,
						 struct pci_epf_nvme_cmd, link);
			list_del_init(&epcmd->link);
			if (!pci_epf_nvme_queue_response(epcmd))
				break;
			nr_cqe++;
		}

		if (nr_cqe && pci_epf_nvme_ctrl_ready(cq->epf_nvme))
			pci_epf_nvme_raise_irq(cq->epf_nvme, cq);

		spin_lock_irqsave(&cq->lock, flags);

		if (!list_empty(&list)) {
			list_splice_tail(&list, &cq->list);
			reschedule = true;
			break;
		}
	}

	spin_unlock_irqrestore(&cq->lock, flags);

	if (reschedule)
		schedule_delayed_work(&cq->work, msecs_to_jiffies(1));
}

static void pci_epf_nvme_drain_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];
	struct pci_epf_nvme_cmd *epcmd;

	if (sq->state != PCI_EPF_NVME_QUEUE_LIVE)
		return;

	flush_delayed_work(&sq->work);
	cancel_delayed_work_sync(&sq->work);

	while ((epcmd = pci_epf_nvme_fetch_cmd(epf_nvme, sq))) {
		epcmd->status = NVME_SC_ABORT_QUEUE | NVME_SC_DNR;
		pci_epf_nvme_complete_cmd(epcmd);
	}
}

static void pci_epf_nvme_drain_cq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *cq = &epf_nvme->ctrl.cq[qid];
	struct pci_epf_nvme_cmd *epcmd;

	if (cq->state != PCI_EPF_NVME_QUEUE_LIVE)
		return;

	flush_delayed_work(&cq->work);
	cancel_delayed_work_sync(&cq->work);

	while (!list_empty(&cq->list)) {
		epcmd = list_first_entry(&cq->list,
					 struct pci_epf_nvme_cmd, link);
		list_del_init(&epcmd->link);
		pci_epf_nvme_free_cmd(epcmd);
	}
}

static int pci_epf_nvme_map_cq(struct pci_epf_nvme *epf_nvme, int qid,
			       int flags, int size, int vector,
			       phys_addr_t pci_addr)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *cq = &ctrl->cq[qid];
	struct pci_epf *epf = epf_nvme->epf;
	size_t qsize;
	ssize_t ret;

	/*
	 * Increment the queue ref count: if the queue was already mapped,
	 * we have nothing to do.
	 */
	cq->ref++;
	if (cq->ref > 1)
		return 0;

	/* Setup and map the completion queue */
	cq->qid = qid;
	cq->size = size;
	cq->flags = flags;
	cq->depth = size + 1;
	cq->vector = vector;
	cq->head = 0;
	cq->tail = 0;
	cq->phase = 1;
	cq->db = NVME_REG_DBS + (((qid * 2) + 1) * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, cq->db, 0);
	INIT_DELAYED_WORK(&cq->work, pci_epf_nvme_cq_work);

	cq->cqe_wq = alloc_workqueue("cq%d_cqe_wq", WQ_HIGHPRI, 1, qid);
	if (!cq->cqe_wq) {
		dev_err(&epf->dev, "Create CQ %d cqe wq failed\n", qid);
		ret = -ENOMEM;
		goto err;
	}

	if (!qid)
		cq->qes = ctrl->adm_cqes;
	else
		cq->qes = ctrl->io_cqes;
	qsize = cq->qes * cq->depth;

	ret = pci_epf_mem_map(epf, pci_addr, qsize, &cq->map);
	if (ret != qsize) {
		dev_err(&epf->dev, "Map CQ %d failed\n", qid);
		if (ret >= 0)
			ret = -ENOMEM;
		goto err;
	}

	dev_dbg(&epf->dev,
		"CQ %d: PCI addr 0x%llx, virt addr 0x%llx, size %zu B\n",
		qid, cq->map.pci_addr, (u64)cq->map.virt_addr, qsize);
	dev_dbg(&epf->dev,
		"CQ %d: %d entries of %zu B, vector IRQ %d\n",
		qid, cq->size, cq->qes, (int)cq->vector + 1);

	cq->state = PCI_EPF_NVME_QUEUE_LIVE;

	return 0;

err:
	if (cq->cqe_wq)
		destroy_workqueue(cq->cqe_wq);
	memset(cq, 0, sizeof(*cq));
	return ret;
}

static void pci_epf_nvme_unmap_cq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *cq = &epf_nvme->ctrl.cq[qid];

	if (cq->ref < 1)
		return;

	cq->ref--;
	if (cq->ref)
		return;

	pci_epf_nvme_drain_cq(epf_nvme, qid);

	cq->state = PCI_EPF_NVME_QUEUE_DEAD;

	flush_workqueue(cq->cqe_wq);
	destroy_workqueue(cq->cqe_wq);
	cq->cmd_wq = NULL;

	pci_epf_mem_unmap(epf_nvme->epf, &cq->map);
}

static void pci_epf_nvme_sq_work(struct work_struct *work);

static int pci_epf_nvme_map_sq(struct pci_epf_nvme *epf_nvme, int qid,
			       int cqid, int flags, int size,
			       phys_addr_t pci_addr)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[cqid];
	struct pci_epf *epf = epf_nvme->epf;
	ssize_t ret = -ENOMEM;
	size_t qsize;

	/* Setup and map the submission queue */
	sq->ref = 1;
	sq->qid = qid;
	sq->cqid = cqid;
	sq->size = size;
	sq->flags = flags;
	sq->depth = size + 1;
	sq->head = 0;
	sq->tail = 0;
	sq->phase = 0;
	sq->db = NVME_REG_DBS + (qid * 2 * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, sq->db, 0);
	INIT_DELAYED_WORK(&sq->work, pci_epf_nvme_sq_work);

	sq->cmd_wq = alloc_workqueue("sq%d_cmd_wq", WQ_HIGHPRI | WQ_UNBOUND,
				     min_t(int, sq->depth, WQ_MAX_ACTIVE), qid);
	if (!sq->cmd_wq) {
		dev_err(&epf->dev, "Create SQ %d cmd wq failed\n", qid);
		ret = -ENOMEM;
		goto err;
	}

	if (!qid)
		sq->qes = ctrl->adm_sqes;
	else
		sq->qes = ctrl->io_sqes;
	qsize = sq->qes * sq->depth;

	ret = pci_epf_mem_map(epf, pci_addr, qsize, &sq->map);
	if (ret != qsize) {
		dev_err(&epf->dev, "Map SQ %d failed\n", qid);
		if (ret >= 0)
			ret = -ENOMEM;
		goto err;
	}

	/* Get a reference on the completion queue */
	cq->ref++;

	dev_dbg(&epf->dev,
		"SQ %d: PCI addr 0x%llx, virt addr 0x%llx, size %zu B\n",
		qid, sq->map.pci_addr, (u64)sq->map.virt_addr, qsize);
	dev_dbg(&epf->dev,
		"SQ %d: %d queue entries of %zu B, CQ %d\n",
		qid, size, sq->qes, cqid);

	sq->state = PCI_EPF_NVME_QUEUE_LIVE;

	return 0;

err:
	if (sq->cmd_wq)
		destroy_workqueue(sq->cmd_wq);
	memset(sq, 0, sizeof(*sq));
	return ret;
}

static void pci_epf_nvme_unmap_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];

	if (!sq->ref)
		return;

	sq->ref--;
	if (WARN_ON_ONCE(sq->ref != 0))
		return;

	pci_epf_nvme_drain_sq(epf_nvme, qid);

	sq->state = PCI_EPF_NVME_QUEUE_DEAD;

	flush_workqueue(sq->cmd_wq);
	destroy_workqueue(sq->cmd_wq);
	sq->cmd_wq = NULL;

	if (epf_nvme->ctrl.cq[sq->cqid].ref)
		epf_nvme->ctrl.cq[sq->cqid].ref--;

	pci_epf_mem_unmap(epf_nvme->epf, &sq->map);
}

static void pci_epf_nvme_disable_ctrl(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	int qid;

	if (!epf_nvme->ctrl_enabled)
		return;

	dev_info(&epf->dev, "Disabling controller\n");

	/*
	 * Unmap the submission queues first to release all references
	 * to the completion queues. This also stops polling for
	 * submissions and drains any pending submission from the queue.
	 */
	for (qid = 1; qid < ctrl->nr_queues; qid++)
		pci_epf_nvme_unmap_sq(epf_nvme, qid);

	for (qid = 1; qid < ctrl->nr_queues; qid++)
		pci_epf_nvme_unmap_cq(epf_nvme, qid);

	/* Unmap the admin queue last */
	pci_epf_nvme_unmap_sq(epf_nvme, 0);
	pci_epf_nvme_unmap_cq(epf_nvme, 0);

	/* Tell the root complex we are done */
	ctrl->csts &= ~NVME_CSTS_RDY;
	if (ctrl->cc & NVME_CC_SHN_NORMAL) {
		ctrl->csts |= NVME_CSTS_SHST_CMPLT;
		ctrl->cc &= ~NVME_CC_SHN_NORMAL;
	}
	ctrl->cc &= ~NVME_CC_ENABLE;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);

	epf_nvme->ctrl_enabled = false;
}

static void pci_epf_nvme_delete_ctrl(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	dev_info(&epf->dev, "Deleting controller\n");

	if (ctrl->ctrl) {
		nvme_put_ctrl(ctrl->ctrl);
		ctrl->ctrl = NULL;

		ctrl->cc &= ~NVME_CC_SHN_NORMAL;
		ctrl->csts |= NVME_CSTS_SHST_CMPLT;
	}

	pci_epf_nvme_disable_ctrl(epf_nvme);

	ctrl->nr_queues = 0;
	kfree(ctrl->cq);
	ctrl->cq = NULL;
	kfree(ctrl->sq);
	ctrl->sq = NULL;
}

static struct pci_epf_nvme_queue *
pci_epf_nvme_alloc_queues(struct pci_epf_nvme *epf_nvme, int nr_queues)
{
	struct pci_epf_nvme_queue *q;
	int i;

	q = kcalloc(nr_queues, sizeof(struct pci_epf_nvme_queue), GFP_KERNEL);
	if (!q)
		return NULL;

	for (i = 0; i < nr_queues; i++) {
		q[i].epf_nvme = epf_nvme;
		spin_lock_init(&q[i].lock);
		INIT_LIST_HEAD(&q[i].list);
	}

	return q;
}

static int pci_epf_nvme_create_ctrl(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_ctrl *fctrl;
	int ret;

	/* We must have nvme fabrics options. */
	if (!epf_nvme->ctrl_opts_buf) {
		dev_err(&epf->dev, "No nvme fabrics options specified\n");
		return -EINVAL;
	}

	/* Create the fabrics controller */
	fctrl = nvmf_create_ctrl(&epf->dev, epf_nvme->ctrl_opts_buf);
	if (IS_ERR(fctrl)) {
		dev_err(&epf->dev, "Create nvme fabrics controller failed\n");
		return PTR_ERR(fctrl);
	}

	/* We only support IO controllers */
	if (fctrl->cntrltype != NVME_CTRL_IO) {
		dev_err(&epf->dev, "Unsupported controller type\n");
		ret = -EINVAL;
		goto out_delete_ctrl;
	}

	dev_info(&epf->dev, "NVMe fabrics controller created, %u I/O queues\n",
		 fctrl->queue_count - 1);

	epf_nvme->queue_count = epf_nvme->qid_max + 1;
	if (features->msix_capable && epf->msix_interrupts) {
		dev_info(&epf->dev,
			 "NVMe PCI controller supports MSIX, %u vectors\n",
			 epf->msix_interrupts);
		epf_nvme->queue_count =
			min(epf_nvme->queue_count, epf->msix_interrupts);
	} else if (features->msi_capable && epf->msi_interrupts) {
		dev_info(&epf->dev,
			 "NVMe PCI controller supports MSI, %u vectors\n",
			 epf->msi_interrupts);
		epf_nvme->queue_count =
			min(epf_nvme->queue_count, epf->msi_interrupts);
	}

	dev_info(&epf->dev, "NVMe PCI controller: %u I/O queues\n",
		 epf_nvme->queue_count - 1);

	/* Allocate queues */
	ctrl->nr_queues = epf_nvme->queue_count;
	ctrl->sq = pci_epf_nvme_alloc_queues(epf_nvme, ctrl->nr_queues);
	if (!ctrl->sq) {
		ret = -ENOMEM;
		goto out_delete_ctrl;
	}

	ctrl->cq = pci_epf_nvme_alloc_queues(epf_nvme, ctrl->nr_queues);
	if (!ctrl->cq) {
		ret = -ENOMEM;
		goto out_delete_ctrl;
	}

	epf_nvme->ctrl.ctrl = fctrl;

	return 0;

out_delete_ctrl:
	pci_epf_nvme_delete_ctrl(epf);

	return ret;
}

static void pci_epf_nvme_init_ctrl_regs(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	ctrl->reg = epf_nvme->reg[epf_nvme->reg_bar];

	/* Copy the fabrics controller capabilities as a base */
	ctrl->cap = ctrl->ctrl->cap;

	/* Contiguous Queues Required (CQR) */
	ctrl->cap |= 0x1ULL << 16;

	/* Set Doorbell stride to 4B (DSTRB) */
	ctrl->cap &= ~GENMASK(35, 32);

	/* Clear NVM Subsystem Reset Supported (NSSRS) */
	ctrl->cap &= ~(0x1ULL << 36);

	/* Clear Boot Partition Support (BPS) */
	ctrl->cap &= ~(0x1ULL << 45);

	/* Memory Page Size minimum (MPSMIN) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Memory Page Size maximum (MPSMAX) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Clear Persistent Memory Region Supported (PMRS) */
	ctrl->cap &= ~(0x1ULL << 56);

	/* Clear Controller Memory Buffer Supported (CMBS) */
	ctrl->cap &= ~(0x1ULL << 57);

	/* NVMe version supported */
	ctrl->vs = ctrl->ctrl->vs;

	/* Controller configuration */
	ctrl->cc = ctrl->ctrl->ctrl_config & (~NVME_CC_ENABLE);

	/* Controller Status (not ready) */
	ctrl->csts = 0;

	pci_epf_nvme_reg_write64(ctrl, NVME_REG_CAP, ctrl->cap);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_VS, ctrl->vs);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);

	return;
}

static void pci_epf_nvme_enable_ctrl(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	int ret;

	dev_info(&epf->dev, "Enabling controller\n");

	ctrl->mdts = epf_nvme->mdts_kb * SZ_1K;

	ctrl->mps_shift = ((ctrl->cc >> NVME_CC_MPS_SHIFT) & 0xf) + 12;
	ctrl->mps = 1UL << ctrl->mps_shift;
	ctrl->mps_mask = ctrl->mps - 1;

	ctrl->adm_sqes = 1UL << NVME_ADM_SQES;
	ctrl->adm_cqes = sizeof(struct nvme_completion);
	ctrl->io_sqes = 1UL << ((ctrl->cc >> NVME_CC_IOSQES_SHIFT) & 0xf);
	ctrl->io_cqes = 1UL << ((ctrl->cc >> NVME_CC_IOCQES_SHIFT) & 0xf);

	if (ctrl->io_sqes < sizeof(struct nvme_command)) {
		dev_err(&epf->dev, "Unsupported IO sqes %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_command));
		return;
	}

	if (ctrl->io_cqes < sizeof(struct nvme_completion)) {
		dev_err(&epf->dev, "Unsupported IO cqes %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_completion));
		return;
	}

	ctrl->aqa = pci_epf_nvme_reg_read32(ctrl, NVME_REG_AQA);
	ctrl->asq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ASQ);
	ctrl->acq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ACQ);

	/*
	 * Map the PCI controller admin submission and completion queues.
	 */
	ret = pci_epf_nvme_map_cq(epf_nvme, 0,
				  NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED,
				  (ctrl->aqa & 0x0fff0000) >> 16, 0,
				  ctrl->acq & GENMASK(63, 12));
	if (ret)
		return;

	ret = pci_epf_nvme_map_sq(epf_nvme, 0, 0, NVME_QUEUE_PHYS_CONTIG,
				  ctrl->aqa & 0x0fff,
				  ctrl->asq & GENMASK(63, 12));
	if (ret) {
		pci_epf_nvme_unmap_cq(epf_nvme, 0);
		return;
	}

	nvme_start_ctrl(ctrl->ctrl);

	/* Tell the host we are now ready */
	ctrl->csts |= NVME_CSTS_RDY;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	/* Start polling the admin submission queue */
	schedule_delayed_work(&ctrl->sq[0].work, msecs_to_jiffies(5));

	epf_nvme->ctrl_enabled = true;

	return;
}

static void pci_epf_nvme_create_cq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	int mqes = NVME_CAP_MQES(epf_nvme->ctrl.cap);
	u16 cqid, cq_flags, qsize, vector;
	int ret;

	cqid = le16_to_cpu(cmd->create_cq.cqid);
	if (cqid >= epf_nvme->ctrl.nr_queues || epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cq_flags = le16_to_cpu(cmd->create_cq.cq_flags);
	if (!(cq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_cq.qsize);
	if (!qsize || qsize > NVME_CAP_MQES(epf_nvme->ctrl.cap)) {
		if (qsize > mqes)
			dev_warn(&epf_nvme->epf->dev,
				 "Create CQ %d, qsize %d > mqes %d: buggy driver?\n",
				 cqid, (int)qsize, mqes);
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	vector = le16_to_cpu(cmd->create_cq.irq_vector);
	if (vector >= epf_nvme->nr_vectors) {
		epcmd->status = NVME_SC_INVALID_VECTOR | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_map_cq(epf_nvme, cqid, cq_flags, qsize, vector,
				  le64_to_cpu(cmd->create_cq.prp1));
	if (ret)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
}

static void pci_epf_nvme_delete_cq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	u16 cqid;

	cqid = le16_to_cpu(cmd->delete_queue.qid);
	if (!cqid ||
	    cqid >= epf_nvme->ctrl.nr_queues ||
	    !epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	pci_epf_nvme_drain_cq(epf_nvme, cqid);
	pci_epf_nvme_unmap_cq(epf_nvme, cqid);
}

static void pci_epf_nvme_create_sq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	int mqes = NVME_CAP_MQES(epf_nvme->ctrl.cap);
	u16 sqid, cqid, sq_flags, qsize;
	int ret;

	sqid = le16_to_cpu(cmd->create_sq.sqid);
	if (!sqid || sqid > epf_nvme->ctrl.nr_queues ||
	    epf_nvme->ctrl.sq[sqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cqid = le16_to_cpu(cmd->create_sq.cqid);
	if (!cqid || !epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_CQ_INVALID | NVME_SC_DNR;
		return;
	}

	sq_flags = le16_to_cpu(cmd->create_sq.sq_flags);
	if (!(sq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_sq.qsize);
	if (!qsize || qsize > mqes) {
		if (qsize > mqes)
			dev_warn(&epf_nvme->epf->dev,
				 "Create SQ %d, qsize %d > mqes %d: buggy driver?\n",
				 sqid, (int)qsize, mqes);
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_map_sq(epf_nvme, sqid, cqid, sq_flags, qsize,
				  le64_to_cpu(cmd->create_sq.prp1));
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		return;
	}

	/* Start polling the submission queue */
	schedule_delayed_work(&epf_nvme->ctrl.sq[sqid].work,
			      msecs_to_jiffies(1));
}

static void pci_epf_nvme_delete_sq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	u16 sqid;

	sqid = le16_to_cpu(cmd->delete_queue.qid);
	if (!sqid ||
	    sqid >= epf_nvme->ctrl.nr_queues ||
	    !epf_nvme->ctrl.sq[sqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	pci_epf_nvme_unmap_sq(epf_nvme, sqid);
}

static void pci_epf_nvme_identify_hook(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct nvme_command *cmd = &epcmd->cmd;
	struct nvme_id_ctrl *id = epcmd->buffer;
	unsigned int page_shift;

	if (cmd->identify.cns != NVME_ID_CNS_CTRL)
		return;

	/* Set device vendor IDs */
	id->vid = cpu_to_le16(epf_nvme->epf->header->vendorid);
	id->ssvid = id->vid;

	/* Set Maximum Data Transfer Size (MDTS) */
	page_shift = NVME_CAP_MPSMIN(epf_nvme->ctrl.ctrl->cap) + 12;
	id->mdts = ilog2(epf_nvme->ctrl.mdts) - page_shift;

	/* Clear Controller Multi-Path I/O and Namespace Sharing Capabilities */
	id->cmic = 0;

	/* Do not report support for Autonomous Power State Transitions */
	id->apsta = 0;

	/* Indicate no support for SGLs */
	id->sgls = 0;

	/* Implementations before 1.4 may report a value of 0h (Fig. 276 Base Specs)*/
	if (((id->ver && GENMASK(31, 16)) >> 16) < cpu_to_le16(2)) {
		if (((id->ver && GENMASK(15, 8)) >> 8) < 4 ) {
			id->cntrltype = 0;
		}
	}

	/* Indicate no support for Asymmetric Namespace Access (ANA) */
	id->anatt = 0;
	id->anacap = 0;
	id->anagrpmax = 0;
	id->nanagrpid = 0;

	/* Maximum outstanding commands is optional for NVMe over PCIe */
	id->maxcmd = 0;

	/* Number of Namespaces should be 1 */
	id->nn = cpu_to_le32(1);
	id->mnan = 0;

	/* Clear fabrics specific values */
	id->ioccsz = 0;
	id->iorcsz = 0;

	/* Set power state info */
	id->psd[0].max_power = cpu_to_le16(700); /* centiwatts */
}

static void pci_epf_nvme_get_log_hook(struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct nvme_effects_log *log = epcmd->buffer;

	if (cmd->get_log_page.lid != NVME_LOG_CMD_EFFECTS)
		return;

	/*
	 * ACS0     [Delete I/O Submission Queue     ] 00000001
	 * CSUPP+  LBCC-  NCC-  NIC-  CCC-  USS-  No command restriction
	 */
	log->acs[0] |= cpu_to_le32(NVME_CMD_EFFECTS_CSUPP);

	/*
	 * ACS1     [Create I/O Submission Queue     ] 00000001
	 * CSUPP+  LBCC-  NCC-  NIC-  CCC-  USS-  No command restriction
	 */
	log->acs[1] |= cpu_to_le32(NVME_CMD_EFFECTS_CSUPP);

	/*
	 * ACS4     [Delete I/O Completion Queue     ] 00000001
	 * CSUPP+  LBCC-  NCC-  NIC-  CCC-  USS-  No command restriction
	 */
	log->acs[4] |= cpu_to_le32(NVME_CMD_EFFECTS_CSUPP);

	/*
	 * ACS5     [Create I/O Completion Queue     ] 00000001
	 * CSUPP+  LBCC-  NCC-  NIC-  CCC-  USS-  No command restriction
	 */
	log->acs[5] |= cpu_to_le32(NVME_CMD_EFFECTS_CSUPP);
}

static bool pci_epf_nvme_process_set_features(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	u32 cdw10 = le32_to_cpu(epcmd->cmd.common.cdw10);
	u32 cdw11 = le32_to_cpu(epcmd->cmd.common.cdw11);
	u8 feat = cdw10 & 0xff;
	u16 nsqr, ncqr;

	if (feat == NVME_FEAT_NUM_QUEUES) {
		ncqr = (cdw11 >> 16) & 0xffff;
		nsqr = cdw11 & 0xffff;
		if (ncqr == 0xffff || nsqr == 0xffff) {
			epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
			return true;
		}

		epcmd->cqe.result.u32 = cpu_to_le32((epf_nvme->qid_max - 1) |
						((epf_nvme->qid_max - 1) << 16));
		return true;
	} else if (feat == NVME_FEAT_IRQ_COALESCE) {
		epcmd->status = NVME_SC_SUCCESS;
		return true;
	} else if (feat == NVME_FEAT_ARBITRATION) {
		epcmd->status = NVME_SC_SUCCESS;
		return true;
	} else
		return false;
}

static void pci_epf_nvme_process_admin_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	void (*post_exec_hook)(struct pci_epf_nvme_cmd *) = NULL;
	struct nvme_command *cmd = &epcmd->cmd;
	//int ret;

	switch (cmd->common.opcode) {
	case nvme_admin_identify:
		post_exec_hook = pci_epf_nvme_identify_hook;
		dev_dbg(&epf_nvme->epf->dev,
			"Command %s, CNS : %d\n",
			pci_epf_nvme_cmd_name(epcmd), cmd->identify.cns);
		epcmd->buffer_size = NVME_IDENTIFY_DATA_SIZE;
		epcmd->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_admin_get_log_page:
		post_exec_hook = pci_epf_nvme_get_log_hook;
		epcmd->buffer_size = nvme_get_log_page_len(cmd);
		epcmd->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_admin_async_event:
		/*
		 * Async events are a pain to deal with as they get canceled
		 * only once we delete the fabrics controller, which happens
		 * after the epf function is deleted, thus causing access to
		 * freed memory or leaking of epcmd. So ignore these commands
		 * for now, which is fine. The host will simply never see any
		 * event.
		 */
		pci_epf_nvme_free_cmd(epcmd);
		return;

	case nvme_admin_set_features:
		dev_dbg(&epf_nvme->epf->dev,
			"Command %s, FID : %#08x\n",
			pci_epf_nvme_cmd_name(epcmd), cmd->features.fid);
		if (pci_epf_nvme_process_set_features(epcmd))
			goto complete;
		break;
	case nvme_admin_get_features:
		dev_dbg(&epf_nvme->epf->dev,
			"Command %s, FID : %#08x\n",
			pci_epf_nvme_cmd_name(epcmd), cmd->features.fid);
	case nvme_admin_abort_cmd:
		break;

	case nvme_admin_create_cq:
		pci_epf_nvme_create_cq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_create_sq:
		pci_epf_nvme_create_sq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_delete_cq:
		pci_epf_nvme_delete_cq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_delete_sq:
		pci_epf_nvme_delete_sq(epf_nvme, epcmd);
		goto complete;

	default:
		dev_err(&epf_nvme->epf->dev,
			"Unhandled admin command %s (0x%02x)\n",
			pci_epf_nvme_cmd_name(epcmd), cmd->common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	/* Synchronously execute the command */
	pci_epf_nvme_exec_cmd(epcmd, post_exec_hook);

complete:
	pci_epf_nvme_complete_cmd(epcmd);
}

static inline size_t pci_epf_nvme_rw_data_len(struct pci_epf_nvme_cmd *epcmd)
{
	return ((u32)le16_to_cpu(epcmd->cmd.rw.length) + 1) <<
		epcmd->ns->head->lba_shift;
}

static void pci_epf_nvme_process_io_cmd(struct pci_epf_nvme_cmd *epcmd,
					struct pci_epf_nvme_queue *sq)
{
	struct nvme_kv_common_command *cmd = (struct nvme_kv_common_command *) 
					      &epcmd->cmd.common;

	if (cmd->opcode & NVME_DIRECTION_TO_HOST_BIT)
		epcmd->dma_dir = DMA_TO_DEVICE;
	if (cmd->opcode & NVME_DIRECTION_FROM_HOST_BIT)
		epcmd->dma_dir = DMA_FROM_DEVICE;

	queue_work(sq->cmd_wq, &epcmd->io_work);

	#if 0
	struct nvme_kv_common_command *cmd = (struct nvme_kv_common_command *) 
					      &epcmd->cmd.common;

	//Get the 8 lowest bits of cdw11
	int key_length = cmd->key_len;
	u64 cdw2_0 = le32_to_cpu(cmd->cdw2[0]);
	u64 cdw2_1 = (u64)le32_to_cpu(cmd->cdw2[1]);
	u64 cdw15 = (u64)(__le32_to_cpu(cmd->cdw15));
	u64 key_lsb = ((cdw2_1 << 32) | cdw2_0);
	u64 key_msb = ((cdw15 << 32) | __le32_to_cpu(cmd->cdw14));
	int ret = 0;
	char temporal_buffer[500];
	loff_t file_offset = 0;
	int value_size = 0;
	int previous_value_size = 0;
	struct file *kv_file = NULL;
	char kv_path[KV_PATH_LEN];
	char *path_key_ptr = kv_path + strlen(KV_BASE_PATH);
	unsigned char bit_to_check = 0;
 	unsigned char mask = ' ';


	/* Get the command target namespace */
	epcmd->ns = nvme_find_get_ns(epf_nvme->ctrl.ctrl,
				     le32_to_cpu(epcmd->cmd.common.nsid));
	if (!epcmd->ns) {
		epcmd->status = NVME_SC_INVALID_NS | NVME_SC_DNR;
		goto complete;
	}

	if (epcmd->cmd.common.opcode == nvme_cmd_kv_store || 
	    epcmd->cmd.common.opcode == nvme_cmd_kv_retrieve ||
	    epcmd->cmd.common.opcode == nvme_cmd_kv_exist ||
	    epcmd->cmd.common.opcode == nvm_cmd_kv_list ||
	    epcmd->cmd.common.opcode == nvme_cmd_kv_delete) {
		
		dev_info(&epf_nvme->epf->dev, "It is a KV Command");
		
		if(key_length > 16 || key_length <= 0) {
			dev_err(&epf_nvme->epf->dev,
				"ERROR: key size NOT valid\n");
			epcmd->status = KV_ERR_INVALID_KEY_SIZE;
			goto complete;
		}
		memset(kv_path, 0, KV_PATH_LEN);
		sprintf(kv_path, KV_BASE_PATH);

		__bin_to_hex((const char*)&key_lsb, 
			     umin((size_t)key_length,sizeof(key_lsb)), 
			     path_key_ptr);
		if (key_length > sizeof(key_lsb)) {
			__bin_to_hex((const char*)&key_msb, 
				     key_length - sizeof(key_lsb), 
				     path_key_ptr + sizeof(key_lsb)*2);
		}
		dev_info(&epf_nvme->epf->dev,
		 	 "Key length: %d Key value: %s\n", key_length,
			 kv_path);
	}

	//This code is for checking if is TO or FROM device
	//It works for both the KV commands and for the I/O
	mask = 1 << bit_to_check;
	if (cmd->opcode & mask) {
		dev_info(&epf_nvme->epf->dev, "Soc un write");
		epcmd->buffer_size = pci_epf_nvme_rw_data_len(epcmd);
		epcmd->dma_dir = DMA_FROM_DEVICE;
	}
	bit_to_check = 1;
	mask = 1 << bit_to_check;
	if (cmd->opcode & mask) {
		dev_info(&epf_nvme->epf->dev, "NO soc un write");
		epcmd->buffer_size = pci_epf_nvme_rw_data_len(epcmd);
		epcmd->dma_dir = DMA_TO_DEVICE;

	}

	switch (epcmd->cmd.common.opcode) {
	//comment this part of the code because we want ot be able to execute
	//commands from both I/O and KV
	/*
	case nvme_cmd_read:
		epcmd->buffer_size = pci_epf_nvme_rw_data_len(epcmd);
		epcmd->dma_dir = DMA_TO_DEVICE;
		break;

	case nvme_cmd_write:
		epcmd->buffer_size = pci_epf_nvme_rw_data_len(epcmd);
		epcmd->dma_dir = DMA_FROM_DEVICE;
		break;
	*/
	//the read & write have the same opcodes than store & retrive so will 
	//work
	//for both IO and KV Commands
	case nvme_cmd_kv_store:

		struct nvme_kv_store_command *store_cmd = 
						(struct nvme_kv_store_command *) 
					      	&epcmd->cmd.common;
		value_size = store_cmd->cdw10;
		if(value_size < 0 || value_size > MAX_NUM_VALUE_SIZE) {
			epcmd->status = KV_ERR_INVALID_VALUE_SIZE;
			goto complete;
		}
		if (file_doesnt_exist(kv_path, epcmd)) {	//insert
			if (value_size > actual_size) {
				epcmd->status = KV_ERR_CAPACITY_EXCEEDED;
				goto complete;
			}
			/*TO DO: if the bit 8 is set to 1*/
			else {
				is_insert = 1;
				kv_file = filp_open(kv_path, O_RDWR | 
						    O_CREAT, 0666);
			}
		}
		else {						//update
			dev_info(&epf_nvme->epf->dev, "Is an UPDATE");
			/*TO DO: if bit 9 is set to 1*/
			previous_value_size = 0;
			ret = 1;
			file_offset = 0;
			kv_file = filp_open(kv_path, O_RDWR | O_CREAT, 0666);
			while(ret != 0) {
				ret = kernel_read(kv_file, temporal_buffer, 
						  500, &file_offset);
				previous_value_size += ret;
			}
			actual_size += previous_value_size;
			if (value_size > actual_size) {
				actual_size -= value_size;
				epcmd->status = KV_ERR_CAPACITY_EXCEEDED;
				goto complete;
			}
			delete_file(kv_path);
			kv_file = filp_open(kv_path, O_RDWR | O_CREAT, 0666);
			dev_info(&epf_nvme->epf->dev, 
				 "kv_store UPDATE %s\n", path_key_ptr);

		}
		break;
	case nvme_cmd_kv_retrieve:
		break;
	case nvme_cmd_dsm:
		epcmd->buffer_size = (le32_to_cpu(epcmd->cmd.dsm.nr) + 1) *
			sizeof(struct nvme_dsm_range);
		epcmd->dma_dir = DMA_FROM_DEVICE;
		goto complete;

	case nvme_cmd_flush:
	case nvme_cmd_write_zeroes:
		break;
	case nvme_cmd_kv_delete:
		epcmd->dma_dir = DMA_NONE;
		if (file_doesnt_exist(kv_path, epcmd)) {
			dev_info(&epf_nvme->epf->dev, 
				"File %s does NOT exist\n", kv_path);
			epcmd->status = KV_ERR_KEY_NOT_EXIST;
			goto complete;
		} else {
			dev_info(&epf_nvme->epf->dev, 
				"File %s does exist\n", kv_path);
			ret = 1;
			value_size = 0;
			while(ret != 0) {
				ret = kernel_read(kv_file, temporal_buffer, 500, &file_offset);
				value_size += ret;
			}
			actual_size += key_length;
			actual_size += value_size;
			delete_file(kv_path);
			epcmd->status = KV_SUCCESS;
			goto complete;
		}

	case nvme_cmd_kv_exist:
		epcmd->dma_dir = DMA_NONE;
		if (file_doesnt_exist(kv_path, epcmd)) {
			dev_info(&epf_nvme->epf->dev, 
				"File %s does NOT exist\n", kv_path);
			epcmd->status = KV_ERR_KEY_NOT_EXIST;
			goto complete;
		}else {
			dev_info(&epf_nvme->epf->dev, 
				"File %s does exist\n", kv_path);
			epcmd->status = KV_SUCCESS;
			goto complete;
		}
	case nvm_cmd_kv_list:
		dev_info(&epf_nvme->epf->dev, "Soc un list");
		break;
	default:
		dev_err(&epf_nvme->epf->dev,
			"Unhandled IO command %s (0x%02x)\n",
			pci_epf_nvme_cmd_name(epcmd),
			epcmd->cmd.common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (cmd->opcode & NVME_DIRECTION_TO_HOST_BIT)
		epcmd->dma_dir = DMA_TO_DEVICE;
	if (cmd->opcode & NVME_DIRECTION_FROM_HOST_BIT)
		epcmd->dma_dir = DMA_FROM_DEVICE;

	queue_work(sq->cmd_wq, &epcmd->io_work);

	/*
	file_offset = 0;

	if (epcmd->nr_segs == 1) {
		if (cmd->opcode == nvme_cmd_kv_store) {
			if (!kv_file || IS_ERR(kv_file)) {
				dev_err(&epf_nvme->epf->dev, 
					"Could not write to file %s\n", 
					kv_path);
			} else {
				dev_info(&epf_nvme->epf->dev, 
						"Writing data to file: %s\n", 
						kv_path);
				DumpHex((void*)epcmd->seg.pci_addr, 
					epcmd->seg.size, epcmd);
				ret = kernel_write(kv_file, 
						   (void*) epcmd->seg.pci_addr, 
						   epcmd->seg.size, 
						   &file_offset);
				if (ret < 0) {
					dev_err(&epf_nvme->epf->dev, 
						"Could not write KV value"
						" to file %s\n", kv_path);
				}
			}
		}
	} else {
		for (int i = 0; i < epcmd->nr_segs; ++i) {
			if (cmd->opcode == nvme_cmd_kv_store) {
				if (!kv_file || IS_ERR(kv_file)) {
					dev_err(&epf_nvme->epf->dev, 
						"Could not write to file %s\n", 
						kv_path);
				} else {
					dev_info(&epf_nvme->epf->dev, 
						"Writing data to file: %s\n",
						kv_path);
					DumpHex((void*)epcmd->segs[i].pci_addr, 
						epcmd->segs[i].size, epcmd);
					ret = kernel_write(kv_file, 
							(void*) 
							epcmd->segs[i].pci_addr, 
							epcmd->segs[i].size, 
							&file_offset);
					if (ret < 0) {
						dev_err(&epf_nvme->epf->dev, 
							"Could not write KV " 
							"value to file %s\n", 
							kv_path);
					}
				}
			}
		}
	}
	*/
	return;

complete:
	pci_epf_nvme_complete_cmd(epcmd);
	#endif
}

static void pci_epf_nvme_sq_work(struct work_struct *work)
{
	struct pci_epf_nvme_queue *sq =
		container_of(work, struct pci_epf_nvme_queue, work.work);
	struct pci_epf_nvme *epf_nvme = sq->epf_nvme;
	struct pci_epf_nvme_cmd *epcmd;
	unsigned long poll_interval = 1;

	/* Process received commands */
	while (pci_epf_nvme_ctrl_ready(epf_nvme)) {
	       epcmd = pci_epf_nvme_fetch_cmd(epf_nvme, sq);
	       if (!epcmd)
		       break;

	       if (sq->qid)
		       pci_epf_nvme_process_io_cmd(epcmd, sq);
	       else
		       pci_epf_nvme_process_admin_cmd(epcmd);
	}

	if (pci_epf_nvme_ctrl_ready(epf_nvme)) {
		if (sq->qid)
			poll_interval = 5;
		schedule_delayed_work(&sq->work,
				      msecs_to_jiffies(poll_interval));
	}
}

static void pci_epf_nvme_reg_poll(struct work_struct *work)
{
	struct pci_epf_nvme *epf_nvme =
		container_of(work, struct pci_epf_nvme, reg_poll.work);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	u32 old_cc;

	/* Set the controller register bar */
	ctrl->reg = epf_nvme->reg[epf_nvme->reg_bar];
	if (!ctrl->reg) {
		dev_err(&epf_nvme->epf->dev, "No register BAR set\n");
		goto again;
	}

	/* Check CC.EN to determine what we need to do */
	old_cc = ctrl->cc;
	ctrl->cc = pci_epf_nvme_reg_read32(ctrl, NVME_REG_CC);

	/* If not enabled yet, wait */
	if (!(old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE))
		goto again;

	/* If CC.EN was set by the host, enbale the controller */
	if (!(old_cc & NVME_CC_ENABLE) && (ctrl->cc & NVME_CC_ENABLE)) {
		pci_epf_nvme_enable_ctrl(epf_nvme);
		goto again;
	}

	/* If CC.EN was cleared by the host, disable the controller */
	if (((old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE)) ||
	    ctrl->cc & NVME_CC_SHN_NORMAL)
		pci_epf_nvme_disable_ctrl(epf_nvme);

again:
	schedule_delayed_work(&epf_nvme->reg_poll, msecs_to_jiffies(5));
}

static int pci_epf_nvme_set_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	enum pci_barno reg_bar = epf_nvme->reg_bar;
	struct pci_epf_bar *epf_bar;
	int bar, add;
	int ret;

	for (bar = BAR_0; bar < BAR_1; bar += add) {
		epf_bar = &epf->bar[bar];

		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation requires a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		if (features->reserved_bar & (1 << bar))
			continue;

		ret = pci_epf_set_bar(epf, epf_bar);
		if (ret) {
			dev_err(&epf->dev, "Failed to set BAR%d\n", bar);
			pci_epf_free_space(epf, epf_nvme->reg[bar], bar,
					   PRIMARY_INTERFACE);
			if (bar == reg_bar)
				return ret;
		}
	}

	return 0;
}

static int pci_epf_nvme_alloc_reg_bar(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	enum pci_barno reg_bar = epf_nvme->reg_bar;
	size_t reg_size, reg_bar_size;
	size_t msix_table_size = 0;

	/*
	 * Calculate the size of the register bar: NVMe registers first with
	 * enough space for the doorbells, followed by the MSIX table
	 * if supported.
	 */
	reg_size = NVME_REG_DBS + (PCI_EPF_NVME_MAX_NR_QUEUES * 2 * sizeof(u32));
	reg_size = ALIGN(reg_size, 8);

	if (features->msix_capable) {
		size_t pba_size;

		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_nvme->msix_table_offset = reg_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);

		reg_size += msix_table_size + pba_size;
	}

	reg_bar_size = ALIGN(reg_size, 4096);

	if (features->bar_fixed_size[reg_bar]) {
		if (reg_bar_size > features->bar_fixed_size[reg_bar]) {
			dev_err(&epf->dev,
				"Reg BAR %d size %llu B too small, need %zu B\n",
				reg_bar,
				features->bar_fixed_size[reg_bar],
				reg_bar_size);
			return -ENOMEM;
		}
		reg_bar_size = features->bar_fixed_size[reg_bar];
	}

	epf_nvme->reg[reg_bar] = pci_epf_alloc_space(epf, reg_bar_size, reg_bar,
						PAGE_SIZE, PRIMARY_INTERFACE);
	if (!epf_nvme->reg[reg_bar]) {
		dev_err(&epf->dev, "Allocate register BAR failed\n");
		return -ENOMEM;
	}
	memset(epf_nvme->reg[reg_bar], 0, reg_bar_size);

	return 0;
}

static int pci_epf_nvme_configure_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	struct pci_epf_bar *epf_bar;
	int bar, add, ret;
	size_t bar_size;

	/* The first free BAR will be our register BAR */
	bar = pci_epc_get_first_free_bar(features);
	if (bar < 0) {
		dev_err(&epf->dev, "No free BAR\n");
		return -EINVAL;
	}
	epf_nvme->reg_bar = bar;

	/* Initialize BAR flags */
	for (bar = BAR_0; bar < BAR_1; bar++) {
		epf_bar = &epf->bar[bar];
		if (features->bar_fixed_64bit & (1 << bar))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	}

	/* Allocate the register BAR */
	ret = pci_epf_nvme_alloc_reg_bar(epf);
	if (ret)
		return ret;

	/* Allocate remaining BARs */
	for (bar = BAR_0; bar < BAR_1; bar += add) {
		epf_bar = &epf->bar[bar];
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		/*
		 * Skip the register BAR (already allocated) and
		 * reserved BARs.
		 */
		if (epf_nvme->reg[bar] || features->reserved_bar & (1 << bar))
			continue;

		bar_size = max_t(size_t, features->bar_fixed_size[bar], SZ_4K);
		epf_nvme->reg[bar] = pci_epf_alloc_space(epf, bar_size, bar,
						PAGE_SIZE, PRIMARY_INTERFACE);
		if (!epf_nvme->reg[bar]) {
			dev_err(&epf->dev, "Allocate BAR%d failed\n", bar);
			return -ENOMEM;
		}

		memset(epf_nvme->reg[bar], 0, bar_size);
	}

	return 0;
}

static void pci_epf_nvme_clear_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	int bar;

	for (bar = BAR_0; bar < BAR_1; bar++) {
		if (!epf_nvme->reg[bar] ||
		    features->reserved_bar & (1 << bar))
			continue;
		pci_epf_clear_bar(epf, &epf->bar[bar]);
		pci_epf_free_space(epf, epf_nvme->reg[bar], bar,
				   PRIMARY_INTERFACE);
		epf_nvme->reg[bar] = NULL;
	}
}

static int pci_epf_nvme_init_irq(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int ret;

	/* Enable MSIX if supported, otherwise, use MSI */
	if (epf_nvme->epc_features->msix_capable && epf->msix_interrupts) {
		ret = pci_epf_set_msix(epf, epf->msix_interrupts,
				       epf_nvme->reg_bar,
				       epf_nvme->msix_table_offset);
		if (ret) {
			dev_err(&epf->dev, "MSI-X configuration failed\n");
			return ret;
		}

		epf_nvme->nr_vectors = epf->msix_interrupts;
		epf_nvme->irq_type = PCI_IRQ_MSIX;

		return 0;
	}

	if (epf_nvme->epc_features->msi_capable && epf->msi_interrupts) {
		ret = pci_epf_set_msi(epf, epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}

		epf_nvme->nr_vectors = epf->msi_interrupts;
		epf_nvme->irq_type = PCI_IRQ_MSI;

		return 0;
	}

	/* MSI and MSIX are not supported. Use INTX */
	epf_nvme->nr_vectors = 1;
	epf_nvme->irq_type = PCI_IRQ_LEGACY;

	return 0;
}

static int pci_epf_nvme_core_init(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int ret;

	if (epf->vfunc_no <= 1) {
		/* Set device ID, class, etc */
		ret = pci_epf_write_header(epf, epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Write configuration header failed %d\n", ret);
			return ret;
		}
	}

	/* Setup the PCIe BARs and enable interrupts */
	ret = pci_epf_nvme_set_bars(epf);
	if (ret)
		return ret;

	ret = pci_epf_nvme_init_irq(epf);
	if (ret)
		return ret;

	pci_epf_nvme_init_ctrl_regs(epf);

	if (!epf_nvme->epc_features->linkup_notifier)
		schedule_delayed_work(&epf_nvme->reg_poll, msecs_to_jiffies(5));

	return 0;
}

static int pci_epf_nvme_link_up(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);

	dev_info(&epf->dev, "Link UP\n");

	pci_epf_nvme_init_ctrl_regs(epf);

	/* Start polling the BAR registers to detect controller enable */
	schedule_delayed_work(&epf_nvme->reg_poll, 0);

	return 0;
}

static int pci_epf_nvme_link_down(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);

	dev_info(&epf->dev, "Link DOWN\n");

	/* Stop polling BAR registers and disable the controller */
	cancel_delayed_work(&epf_nvme->reg_poll);
	pci_epf_nvme_disable_ctrl(epf_nvme);

	return 0;
}

static const struct pci_epf_event_ops pci_epf_nvme_event_ops = {
	.core_init = pci_epf_nvme_core_init,
	.link_up = pci_epf_nvme_link_up,
	.link_down = pci_epf_nvme_link_down,
};

static int pci_epf_nvme_bind(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	bool dma_supported;
	int ret;

	if (!epc) {
		dev_err(&epf->dev, "No endpoint controller\n");
		return -EINVAL;
	}

	epc_features = pci_epf_get_features(epf);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}
	epf_nvme->epc_features = epc_features;

	ret = pci_epf_nvme_configure_bars(epf);
	if (ret)
		return ret;

	if (epf_nvme->dma_enable) {
		dma_supported = pci_epf_nvme_init_dma(epf_nvme);
		if (dma_supported) {
			dev_info(&epf->dev, "DMA supported\n");
		} else {
			dev_info(&epf->dev,
				 "DMA not supported, falling back to mmio\n");
			epf_nvme->dma_enable = false;
		}
	} else {
		dev_info(&epf->dev, "DMA disabled\n");
	}

	/* Create the fabrics host controller */
	ret = pci_epf_nvme_create_ctrl(epf);
	if (ret)
		goto clean_dma;

	if (!epc_features->core_init_notifier) {
		ret = pci_epf_nvme_core_init(epf);
		if (ret)
			goto clean_dma;
	}

	if (!epc_features->linkup_notifier && !epc_features->core_init_notifier)
		schedule_delayed_work(&epf_nvme->reg_poll, msecs_to_jiffies(5));

	return 0;

clean_dma:
	pci_epf_nvme_clean_dma(epf_nvme);
	pci_epf_nvme_clear_bars(epf);

	return ret;
}

static void pci_epf_nvme_unbind(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);

	cancel_delayed_work(&epf_nvme->reg_poll);

	pci_epf_nvme_delete_ctrl(epf);

	pci_epf_nvme_clean_dma(epf_nvme);

	pci_epf_nvme_clear_bars(epf);
}

static struct pci_epf_header epf_nvme_pci_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.progif_code	= 0x02, /* NVM Express */
	.baseclass_code = PCI_BASE_CLASS_STORAGE,
	.subclass_code	= 0x08, /* Non-Volatile Memory controller */
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static int pci_epf_nvme_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct pci_epf_nvme *epf_nvme;

	epf_nvme = devm_kzalloc(&epf->dev, sizeof(*epf_nvme), GFP_KERNEL);
	if (!epf_nvme)
		return -ENOMEM;

	epf_nvme->epf = epf;
	INIT_DELAYED_WORK(&epf_nvme->reg_poll, pci_epf_nvme_reg_poll);

	epf_nvme->prp_list_buf = devm_kzalloc(&epf->dev, NVME_CTRL_PAGE_SIZE,
					      GFP_KERNEL);
	if (!epf_nvme->prp_list_buf)
		return -ENOMEM;

	/* Set default attribute values */
	epf_nvme->dma_enable = true;
	epf_nvme->mdts_kb = PCI_EPF_NVME_MDTS_KB;
	epf_nvme->qid_max = PCI_EPF_NVME_QID_MAX;

	epf->event_ops = &pci_epf_nvme_event_ops;
	epf->header = &epf_nvme_pci_header;
	epf_set_drvdata(epf, epf_nvme);

	return 0;
}

#define to_epf_nvme(epf_group)	\
	container_of((epf_group), struct pci_epf_nvme, group)

static ssize_t pci_epf_nvme_ctrl_opts_show(struct config_item *item,
					   char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	if (!epf_nvme->ctrl_opts_buf)
		return 0;

	return sysfs_emit(page, "%s\n", epf_nvme->ctrl_opts_buf);
}

#define PCI_EPF_NVME_OPT_HIDDEN_NS	"hidden_ns"

static ssize_t pci_epf_nvme_ctrl_opts_store(struct config_item *item,
					    const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	size_t opt_buf_size;

	/* Do not allow setting options when the function is already started */
	if (epf_nvme->ctrl.ctrl)
		return -EBUSY;

	if (!len)
		return -EINVAL;

	if (epf_nvme->ctrl_opts_buf)
		kfree(epf_nvme->ctrl_opts_buf);

	/*
	 * Make sure we have enough room to add the hidden_ns option
	 * if it is missing.
	 */
	opt_buf_size = len + strlen(PCI_EPF_NVME_OPT_HIDDEN_NS) + 2;
	epf_nvme->ctrl_opts_buf = kzalloc(opt_buf_size, GFP_KERNEL);
	if (!epf_nvme->ctrl_opts_buf)
		return -ENOMEM;

	strcpy(epf_nvme->ctrl_opts_buf, page);
	if (!strnstr(page, PCI_EPF_NVME_OPT_HIDDEN_NS, len))
		strncat(epf_nvme->ctrl_opts_buf,
			"," PCI_EPF_NVME_OPT_HIDDEN_NS, opt_buf_size);

	dev_dbg(&epf_nvme->epf->dev,
		"NVMe fabrics controller options: %s\n",
		epf_nvme->ctrl_opts_buf);

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, ctrl_opts);

static ssize_t pci_epf_nvme_dma_enable_show(struct config_item *item,
					    char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%d\n", epf_nvme->dma_enable);
}

static ssize_t pci_epf_nvme_dma_enable_store(struct config_item *item,
					     const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	int ret;

	if (epf_nvme->ctrl_enabled)
		return -EBUSY;

	ret = kstrtobool(page, &epf_nvme->dma_enable);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, dma_enable);

static ssize_t pci_epf_nvme_qid_max_show(struct config_item *item, char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%u\n", epf_nvme->qid_max);
}

static ssize_t pci_epf_nvme_qid_max_store(struct config_item *item,
					  const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	int ret;

	if (epf_nvme->ctrl_enabled)
		return -EBUSY;

	ret = kstrtouint(page, 0, &epf_nvme->qid_max);
	if (ret)
		return ret;
	if (!epf_nvme->qid_max)
		epf_nvme->qid_max = PCI_EPF_NVME_QID_MAX;
	epf_nvme->qid_max =
		min(epf_nvme->qid_max, PCI_EPF_NVME_MAX_NR_QUEUES - 1);

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, qid_max);

static ssize_t pci_epf_nvme_mdts_kb_show(struct config_item *item, char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%zu\n", epf_nvme->mdts_kb);
}

static ssize_t pci_epf_nvme_mdts_kb_store(struct config_item *item,
					  const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	unsigned long mdts_kb;
	int ret;

	if (epf_nvme->ctrl_enabled)
		return -EBUSY;

	ret = kstrtoul(page, 0, &mdts_kb);
	if (ret)
		return ret;
	if (!mdts_kb)
		mdts_kb = PCI_EPF_NVME_MDTS_KB;
	else if (mdts_kb > PCI_EPF_NVME_MAX_MDTS_KB)
		mdts_kb = PCI_EPF_NVME_MAX_MDTS_KB;

	if (!is_power_of_2(mdts_kb))
		return -EINVAL;

	epf_nvme->mdts_kb = mdts_kb;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, mdts_kb);

static struct configfs_attribute *pci_epf_nvme_attrs[] = {
	&pci_epf_nvme_attr_ctrl_opts,
	&pci_epf_nvme_attr_dma_enable,
	&pci_epf_nvme_attr_qid_max,
	&pci_epf_nvme_attr_mdts_kb,
	NULL,
};

static const struct config_item_type pci_epf_nvme_group_type = {
	.ct_attrs	= pci_epf_nvme_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *pci_epf_nvme_add_cfs(struct pci_epf *epf,
						 struct config_group *group)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);

	/* Add the NVMe target attributes */
	config_group_init_type_name(&epf_nvme->group, "nvme",
				    &pci_epf_nvme_group_type);

	return &epf_nvme->group;
}

static const struct pci_epf_device_id pci_epf_nvme_ids[] = {
	{ .name = "pci_epf_nvme" },
	{},
};

static struct pci_epf_ops pci_epf_nvme_ops = {
	.bind	= pci_epf_nvme_bind,
	.unbind	= pci_epf_nvme_unbind,
	.add_cfs = pci_epf_nvme_add_cfs,
};

static struct pci_epf_driver epf_nvme_driver = {
	.driver.name	= "pci_epf_nvme",
	.probe		= pci_epf_nvme_probe,
	.id_table	= pci_epf_nvme_ids,
	.ops		= &pci_epf_nvme_ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_nvme_init(void)
{
	int ret;

	epf_nvme_cmd_cache = kmem_cache_create("epf_nvme_cmd",
					sizeof(struct pci_epf_nvme_cmd),
					0, SLAB_HWCACHE_ALIGN, NULL);
	if (!epf_nvme_cmd_cache)
		return -ENOMEM;

	ret = pci_epf_register_driver(&epf_nvme_driver);
	if (ret)
		goto out_cache;

	pr_info("Registered nvme EPF driver\n");

	return 0;

out_cache:
	kmem_cache_destroy(epf_nvme_cmd_cache);

	pr_err("Register nvme EPF driver failed\n");

	return ret;
}
module_init(pci_epf_nvme_init);

static void __exit pci_epf_nvme_exit(void)
{
	pci_epf_unregister_driver(&epf_nvme_driver);

	kmem_cache_destroy(epf_nvme_cmd_cache);

	pr_info("Unregistered nvme EPF driver\n");
}
module_exit(pci_epf_nvme_exit);

MODULE_DESCRIPTION("PCI endpoint NVMe function driver");
MODULE_AUTHOR("Damien Le Moal <dlemoal@kernel.org>");
MODULE_AUTHOR("Rick Wertenbroek <rick.wertenbroek@gmail.com>");
MODULE_IMPORT_NS(NVME_TARGET_PASSTHRU);
MODULE_IMPORT_NS(NVME_FABRICS);
MODULE_LICENSE("GPL v2");
