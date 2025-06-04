// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro SATA DMA Controller driver
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Base on:
 *	xilinx_dma
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/bits.h>
#include <linux/dmapool.h>
#include <linux/dma-mapping.h>
#include <linux/iopoll.h>

#include "virt-dma.h"

#define SDMA_CHN_SZ		0x30
#define SDMA_CR			0x00
#define  SDMA_CR_FR_CNT(cnt)	((cnt) << 16)
#define  SDMA_CR_RESET		BIT(2)
#define  SDMA_CR_RUN		BIT(0)
#define  SDMA_CR_STOP		0
#define SDMA_SR			0x04
#define  SDMA_SR_HALTED		BIT(0)
#define SDMA_CDESC		0x08
#define SDMA_TDESC		0x10
#define SDMA_PHYS		0x18
#define SDMA_LENGTH		0x28

#define SDMA_XR_IRQ_ERROR	BIT(14)
#define SDMA_XR_IRQ_IOC		BIT(12)
#define SDMA_XR_ALL_IRQ		(SDMA_XR_IRQ_IOC | \
				SDMA_XR_IRQ_ERROR)
#define SDMA_CR_EN		(SDMA_CR_FR_CNT(1) | \
				SDMA_XR_ALL_IRQ | \
				SDMA_CR_RUN)

enum gxmicro_sdma_dir {
	SDMA_MM2S,
	SDMA_S2MM,
	SDMA_CHAN,
};

#define SDMA_SGD_SOF	BIT(27)
#define SDMA_SGD_EOF	BIT(26)
#define SDMA_SGD_NUM 	SZ_128	/* Same as sata_gxmicro: sg_tablesize */

struct gxmicro_sdma_lli {
	dma_addr_t ndesc;
	dma_addr_t addrs;
	u64 reserved;
	u32 control;
	u32 status;
	u32 app0;
	u32 app1;
	u32 app2;
	u32 app3;
	u32 app4; /* Last field used by HW */
	struct list_head llist;
	dma_addr_t phys;
} __aligned(0x40);

struct gxmicro_sdma_desc {
	struct virt_dma_desc vdesc;

	struct list_head llist;
	struct gxmicro_sdma_chan *gchan;
};

struct gxmicro_sdma_chan {
	struct device *dev;
	void __iomem *mmio;

	struct virt_dma_chan vchan;

	struct gxmicro_sdma_lli *llv;
	dma_addr_t llp;
	spinlock_t plock;
	struct list_head plist;

	enum gxmicro_sdma_dir dir;
	bool idle;
};

struct gxmicro_sdma_dev {
	struct dma_device ddev;

	struct gxmicro_sdma_chan gchan[SDMA_CHAN];
};

static inline u32 gxmicro_read(struct gxmicro_sdma_chan *gchan, u32 reg)
{
	return ioread32(gchan->mmio + reg);
}

static inline void gxmicro_write(struct gxmicro_sdma_chan *gchan, u32 reg, u32 val)
{
	iowrite32(val, gchan->mmio + reg);
}

#if 0
static void gxmicro_dump_desc(struct gxmicro_sdma_desc *gdesc)
{
	struct gxmicro_sdma_lli *glli;
	int i = 0;

	pr_info("===== Descriptor Start =====\n");
	list_for_each_entry(glli, &gdesc->llist, llist) {
		pr_info("===== Descriptor: %d =====\n"
			"    Current Descriptor Pointer: 0x%llx\n"
			"    Next Descriptor Pointer: 0x%llx\n"
			"    Buffer Address: 0x%llx\n"
			"    Control: 0x%x\n"
			"    Status: 0x%x\n"
			"    APP0: 0x%x\n"
			"    APP1: 0x%x\n"
			"    APP2: 0x%x\n"
			"    APP3: 0x%x\n"
			"    APP4: 0x%x\n",
			i, glli->phys, glli->ndesc, glli->addrs,
			glli->control, glli->status,
			glli->app0, glli->app1, glli->app2, glli->app3, glli->app4);
		i++;
	}

	pr_info("===== Descriptor End =====\n");
}
#endif

static inline struct gxmicro_sdma_chan *to_gxmicro_sdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct gxmicro_sdma_chan, vchan.chan);
}

static inline struct gxmicro_sdma_desc *to_gxmicro_sdma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct gxmicro_sdma_desc, vdesc);
}

static void gxmicro_sdma_chan_start(struct gxmicro_sdma_chan *gchan)
{
	gxmicro_write(gchan, SDMA_CR, SDMA_CR_EN);

	gchan->idle = false;
}

static void gxmicro_sdma_chan_stop(struct gxmicro_sdma_chan *gchan)
{
	int ret;
	u32 val;

	gxmicro_write(gchan, SDMA_CR, SDMA_CR_RESET);
	ret = readl_poll_timeout(gchan->mmio + SDMA_CR, val,
			!(val & SDMA_CR_RESET), 1000, 50000);
	if (ret)
		dev_err(gchan->dev, "DMA reset timeout!\n");

	gchan->idle = true;
}

/* ****************************** SDMA ****************************** */

static struct gxmicro_sdma_lli *
gxmicro_sdma_alloc_lli(struct gxmicro_sdma_desc *gdesc)
{
	struct gxmicro_sdma_chan *gchan = gdesc->gchan;
	struct gxmicro_sdma_lli *glli = NULL;
	unsigned long flags;

	spin_lock_irqsave(&gchan->plock, flags);
	if (!list_empty(&gchan->plist)) {
		glli = list_first_entry(&gchan->plist, struct gxmicro_sdma_lli, llist);
		list_move_tail(&glli->llist, &gdesc->llist);
	}
	spin_unlock_irqrestore(&gchan->plock, flags);

	return glli;
}

static void gxmicro_sdma_free_lli(struct gxmicro_sdma_desc *gdesc)
{
	struct gxmicro_sdma_chan *gchan = gdesc->gchan;
	unsigned long flags;

	spin_lock_irqsave(&gchan->plock, flags);
	list_splice_tail(&gdesc->llist, &gchan->plist);
	spin_unlock_irqrestore(&gchan->plock, flags);
}

static struct gxmicro_sdma_desc *gxmicro_sdma_alloc_desc(struct gxmicro_sdma_chan *gchan)
{
	struct gxmicro_sdma_desc *gdesc;

	gdesc = kzalloc(sizeof(struct gxmicro_sdma_desc), GFP_KERNEL);
	if (!gdesc)
		return NULL;

	gdesc->gchan = gchan;
	INIT_LIST_HEAD(&gdesc->llist);

	return gdesc;
}

static void gxmicro_sdma_free_desc(struct gxmicro_sdma_desc *gdesc)
{
	gxmicro_sdma_free_lli(gdesc);

	kfree(gdesc);
}

static void vchan_free_desc(struct virt_dma_desc *vdesc)
{
	gxmicro_sdma_free_desc(to_gxmicro_sdma_desc(vdesc));
}

static void gxmicro_sdma_xfer(struct gxmicro_sdma_chan *gchan)
{
	struct gxmicro_sdma_desc *gdesc;
	struct virt_dma_desc *vdesc;
	struct gxmicro_sdma_lli *head, *tail;

	vdesc = vchan_next_desc(&gchan->vchan);
	if (!vdesc)
		return ;

	gdesc = to_gxmicro_sdma_desc(vdesc);
	head = list_first_entry(&gdesc->llist, struct gxmicro_sdma_lli, llist);
	tail = list_last_entry(&gdesc->llist, struct gxmicro_sdma_lli, llist);

	gxmicro_write(gchan, SDMA_CDESC, head->phys);
	gxmicro_sdma_chan_start(gchan);
	gxmicro_write(gchan, SDMA_TDESC, tail->phys);
}

static irqreturn_t gxmicro_sdma_irq(int irq, void *data)
{
	struct gxmicro_sdma_chan *gchan = data;
	struct virt_dma_desc *vdesc;
	unsigned long flags;
	u32 sr;

	sr = gxmicro_read(gchan, SDMA_SR);
	if (!(sr & SDMA_XR_ALL_IRQ))
		return IRQ_NONE;

	gxmicro_write(gchan, SDMA_SR, sr);

	if (sr & SDMA_XR_IRQ_ERROR)
		dev_err(gchan->dev, "Channel has errors 0x%x\n", sr);

	if (sr & SDMA_XR_IRQ_IOC) {
		spin_lock_irqsave(&gchan->vchan.lock, flags);

		gchan->idle = true;

		vdesc = vchan_next_desc(&gchan->vchan);
		if (vdesc) {
			list_del(&vdesc->node);
			vchan_cookie_complete(vdesc);
		}

		gxmicro_sdma_xfer(gchan);

		spin_unlock_irqrestore(&gchan->vchan.lock, flags);
	}

	return IRQ_HANDLED;
}

static int gxmicro_sdma_init(struct platform_device *pdev)
{
	struct gxmicro_sdma_dev *gdev = platform_get_drvdata(pdev);
	struct dma_device *ddev = &gdev->ddev;
	struct gxmicro_sdma_chan *gchan;
	void __iomem *mmio;
	int irq;
	int ch;
	int ret;

	mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mmio))
		return PTR_ERR(mmio);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret < 0) {
		dev_err(&pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	INIT_LIST_HEAD(&ddev->channels);
	for (ch = 0; ch < SDMA_CHAN; ch++) {
		gchan = &gdev->gchan[ch];

		gchan->dir = ch;
		gchan->mmio = mmio + (ch * SDMA_CHN_SZ);
		gchan->dev = &pdev->dev;

		irq = platform_get_irq(pdev, ch);
		if (irq < 0)
			return irq;

		ret = devm_request_irq(&pdev->dev, irq, gxmicro_sdma_irq,
				IRQF_SHARED, dev_name(&pdev->dev), gchan);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request sdma chan%d irq\n", ch);
			return ret;
		}

		spin_lock_init(&gchan->plock);
		gchan->vchan.desc_free = vchan_free_desc;
		vchan_init(&gchan->vchan, ddev);
	}

	return 0;
}

/* ****************************** DMA ****************************** */

static int gxmicro_sdma_alloc_chan_resources(struct dma_chan *chan)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);
	int i;

	gchan->llv = dma_alloc_coherent(gchan->dev, sizeof(struct gxmicro_sdma_lli) * SDMA_SGD_NUM,
					&gchan->llp, GFP_KERNEL);
	if (!gchan->llv) {
		dev_err(gchan->dev, "No memory for descriptors\n");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&gchan->plist);
	for (i = 0; i < SDMA_SGD_NUM; i++) {
		gchan->llv[i].ndesc = gchan->llp +
				sizeof(struct gxmicro_sdma_lli) * ((i + 1) % SDMA_SGD_NUM);
		gchan->llv[i].phys = gchan->llp + sizeof(struct gxmicro_sdma_lli) * i;
		list_add_tail(&gchan->llv[i].llist, &gchan->plist);
	}

	gchan->idle = true;

	return 0;
}

static void gxmicro_sdma_free_chan_resources(struct dma_chan *chan)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);

	gxmicro_sdma_chan_stop(gchan);

	vchan_free_chan_resources(&gchan->vchan);

	dma_free_coherent(gchan->dev,
			sizeof(struct gxmicro_sdma_lli) * SDMA_SGD_NUM,
			gchan->llv, gchan->llp);
}

static int gxmicro_sdma_config(struct dma_chan *chan,
				struct dma_slave_config *config)
{
	return 0;
}

static struct dma_async_tx_descriptor *
gxmicro_sdma_prep_slave_sg(struct dma_chan *chan, struct scatterlist *sgl,
			unsigned int nents, enum dma_transfer_direction direction,
			unsigned long flags, void *context)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);
	struct gxmicro_sdma_desc *gdesc;
	struct gxmicro_sdma_lli *glli, *head, *tail;
	struct scatterlist *sg;
	int i;

	if (sgl == NULL || nents < 1) {
		dev_err(gchan->dev, "Failed to get scatterlist\n");
		return NULL;
	}

	if (!is_slave_direction(direction))
		return NULL;

	gdesc = gxmicro_sdma_alloc_desc(gchan);
	if (!gdesc)
		return NULL;

	for_each_sg(sgl, sg, nents, i) {
		glli = gxmicro_sdma_alloc_lli(gdesc);
		if (!glli)
			goto err_sdma_prep_sg;

		glli->addrs = sg_dma_address(sg);
		glli->control = sg_dma_len(sg);
		glli->status = 0;
#if 0
		if (sg_dma_len(sg) > SZ_32M) {
			dev_err(gchan->dev, "DMA does not exceed 32M per transfer");
			goto err_sdma_prep_sg;
		}
#endif
	}

	head = list_first_entry(&gdesc->llist, struct gxmicro_sdma_lli, llist);
	tail = list_last_entry(&gdesc->llist, struct gxmicro_sdma_lli, llist);
	head->control |= SDMA_SGD_SOF;
	tail->control |= SDMA_SGD_EOF;

	return vchan_tx_prep(&gchan->vchan, &gdesc->vdesc, flags);

err_sdma_prep_sg:
	gxmicro_sdma_free_desc(gdesc);
	return NULL;
}

static int gxmicro_sdma_terminate_all(struct dma_chan *chan)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);
	LIST_HEAD(head);
	unsigned long flags;

	gxmicro_sdma_chan_stop(gchan);

	spin_lock_irqsave(&gchan->vchan.lock, flags);
	vchan_get_all_descriptors(&gchan->vchan, &head);
	spin_unlock_irqrestore(&gchan->vchan.lock, flags);

	vchan_dma_desc_free_list(&gchan->vchan, &head);

	return 0;
}

static void gxmicro_sdma_synchronize(struct dma_chan *chan)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);

	vchan_synchronize(&gchan->vchan);
}

static void gxmicro_sdma_issue_pending(struct dma_chan *chan)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&gchan->vchan.lock, flags);
	if (vchan_issue_pending(&gchan->vchan))
		if (gchan->idle)
			gxmicro_sdma_xfer(gchan);
	spin_unlock_irqrestore(&gchan->vchan.lock, flags);
}

static const struct dma_slave_map gxmicro_sdma_slave_map[] = {
	{"sata_gxmicro.0", "mm2s", (void *)SDMA_MM2S},
	{"sata_gxmicro.0", "s2mm",  (void *)SDMA_S2MM},
};

static bool gxmicro_sdma_filter(struct dma_chan *chan, void *param)
{
	struct gxmicro_sdma_chan *gchan = to_gxmicro_sdma_chan(chan);
	enum gxmicro_sdma_dir dir = (enum gxmicro_sdma_dir)param;

	return gchan->dir == dir;
}

static int gxmicro_dma_init(struct platform_device *pdev)
{
	struct gxmicro_sdma_dev *gdev = platform_get_drvdata(pdev);
	struct dma_device *ddev = &gdev->ddev;
	int ret;

	/* Set DMA channel capabilities */
	dma_cap_zero(ddev->cap_mask);
	dma_cap_set(DMA_SLAVE, ddev->cap_mask);
	dma_cap_set(DMA_PRIVATE, ddev->cap_mask);

	ddev->filter.fn = gxmicro_sdma_filter;
	ddev->filter.mapcnt = ARRAY_SIZE(gxmicro_sdma_slave_map);
	ddev->filter.map = gxmicro_sdma_slave_map;
	ddev->directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV);
	ddev->src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	ddev->dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	ddev->residue_granularity = DMA_RESIDUE_GRANULARITY_DESCRIPTOR;

	/* Set DMA channel callbacks */
	ddev->dev = &pdev->dev;
	ddev->device_alloc_chan_resources = gxmicro_sdma_alloc_chan_resources;
	ddev->device_free_chan_resources = gxmicro_sdma_free_chan_resources;

	ddev->device_prep_slave_sg = gxmicro_sdma_prep_slave_sg;

	ddev->device_config = gxmicro_sdma_config;
	ddev->device_terminate_all = gxmicro_sdma_terminate_all;
	ddev->device_synchronize = gxmicro_sdma_synchronize;
	ddev->device_tx_status = dma_cookie_status;
	ddev->device_issue_pending = gxmicro_sdma_issue_pending;

	dma_set_max_seg_size(ddev->dev, SZ_32M);

	ret = dma_async_device_register(ddev);
	if (ret) {
		dev_err(&pdev->dev, "Falied to register dma device\n");
		return ret;
	}

	return 0;
}

static void gxmicro_dma_fini(struct platform_device *pdev)
{
	struct gxmicro_sdma_dev *gdev = platform_get_drvdata(pdev);
	struct dma_device *ddev = &gdev->ddev;

	dma_async_device_unregister(ddev);
}

/* ****************************** Platform Probe & Remove ****************************** */

static int gxmicro_sdma_probe(struct platform_device *pdev)
{
	struct gxmicro_sdma_dev *gdev;
	int ret;

	gdev = devm_kzalloc(&pdev->dev, sizeof(struct gxmicro_sdma_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, gdev);

	ret = gxmicro_sdma_init(pdev);
	if (ret)
		return ret;

	ret = gxmicro_dma_init(pdev);
	if (ret)
		return ret;

	return 0;
}

static int gxmicro_sdma_remove(struct platform_device *pdev)
{
	gxmicro_dma_fini(pdev);

	return 0;
}

static const struct platform_device_id gxmicro_sdma_id[] = {
	{ .name = KBUILD_MODNAME },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(platform, gxmicro_sdma_id);

static struct platform_driver gxmicro_sdma_drv = {
	.probe = gxmicro_sdma_probe,
	.remove = gxmicro_sdma_remove,
	.driver = {
		.name	= KBUILD_MODNAME,
	},
	.id_table = gxmicro_sdma_id,
};
module_platform_driver(gxmicro_sdma_drv);

MODULE_DESCRIPTION("GXMicro SATA DMA Controller driver");
MODULE_AUTHOR("Dongxiong Zheng <zhengdongxiong@gxmicro.cn>");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL v2");
