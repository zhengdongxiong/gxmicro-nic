// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro ATA
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/completion.h>

#include "gxmicro_sata.h"

#define ATA_ID_DWORD	SZ_128

#if 0
static void gxmicro_dump_tf_fis(struct ata_taskfile *tf)
{
	pr_info("ATA Task File   <-----> FIS\n"
		"    command     <----->     Command(7:0)  : 0x%02x\n"
		"    feature     <----->     Features(7:0) : 0x%02x\n"
		"    lbal        <----->     LBA(7:0)      : 0x%02x\n"
		"    lbam        <----->     LBA(15:8)     : 0x%02x\n"
		"    lbah        <----->     LBA(23:16)    : 0x%02x\n"
		"    device      <----->     Device(7:0)   : 0x%02x\n"
		"    hob_lbal    <----->     LBA(31:24)    : 0x%02x\n"
		"    hob_lbam    <----->     LBA(39:32)    : 0x%02x\n"
		"    hob_lbah    <----->     LBA(47:40)    : 0x%02x\n"
		"    hob_feature <----->     Features(15:8): 0x%02x\n"
		"    nsect       <----->     Count(7:0)    : 0x%02x\n"
		"    hob_nsect   <----->     Count(15:8)   : 0x%02x\n"
		"    NULL        <----->     ICC(7:0)      : 0x00\n"
		"    ctl         <----->     Control(7:0)  : 0x%02x\n"
		"    auxiliary   <----->     Auxiliary     : 0x%x\n"
		"    flags       <----->     NULL          : 0x%lx\n"
		"    protocol    <----->     NULL          : 0x%02x\n",
		tf->command, tf->feature,
		tf->lbal, tf->lbam, tf->lbah, tf->device,
		tf->hob_lbal, tf->hob_lbam, tf->hob_lbah, tf->hob_feature,
		tf->nsect, tf->hob_nsect, tf->ctl, tf->auxiliary,
		tf->flags, tf->protocol);
}
#endif

static void gxmicro_qc_complete(struct gxmicro_sata_dev *gdev)
{
	struct ata_host *host = gdev->host;
	struct ata_port *ap = host->ports[0];
	struct ata_queued_cmd *qc;

	spin_lock(&host->lock);

	qc = ata_qc_from_tag(ap, ap->link.active_tag);
	ata_qc_complete(qc);

	spin_unlock(&host->lock);
}

static void gxmicro_dma_complete(void *dma_async_param)
{
	struct gxmicro_sata_dev *gdev = dma_async_param;
	u32 val;

	do {
		val = gxmicro_read(gdev, SATA_SR);
	} while (val != SATA_SR_DONE);

	gxmicro_write(gdev, SATA_SR, SATA_SR_CLR);

	gxmicro_qc_complete(gdev);
}

static void gxmicro_simulate_complete(struct work_struct *work)
{
	struct gxmicro_sata_dev *gdev = to_gxmicro_sata_dev(work);

	gxmicro_qc_complete(gdev);
}

static enum ata_completion_errors gxmicro_qc_prep(struct ata_queued_cmd *qc)
{
	return AC_ERR_OK;
}

static unsigned int gxmicro_issue_cmd_rw(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct ata_taskfile *tf = &qc->tf;
	struct gxmicro_sata_dev *gdev = ap->private_data;
	struct dma_chan *chan;
	struct dma_async_tx_descriptor *desc;
	u32 lbal, lbah, nsect, cmd;
	u8 lba28;

	if (tf->flags & ATA_TFLAG_LBA48)
		lba28 = tf->hob_lbal;
	else
		lba28 = tf->device & 0xf;

	lbal = SATA_FIS(lba28, tf->lbah, tf->lbam, tf->lbal);
	lbah = SATA_FIS(SATA_NOOP, SATA_NOOP, tf->hob_lbah, tf->hob_lbam);
	nsect = SATA_FIS(SATA_NOOP, SATA_NOOP, tf->hob_nsect, tf->nsect);

	if (nsect == SATA_NOOP) {
		if (tf->flags & ATA_TFLAG_LBA48)
			nsect = SZ_64K;
		else
			nsect = SZ_256;
	}

	switch (qc->dma_dir) {
	case DMA_TO_DEVICE:
		cmd = SATA_CMD_WR(nsect);
		chan = gdev->mm2s;
		break;
	case DMA_FROM_DEVICE:
		cmd = SATA_CMD_RD(nsect);
		chan = gdev->s2mm;
		break;
	default:
		return AC_ERR_INVALID;
	}

	desc = dmaengine_prep_slave_sg(chan, qc->sg, qc->n_elem, qc->dma_dir, 0);
	if (!desc) {
		ata_port_err(ap, "Failed to prep dma desc");
		return AC_ERR_SYSTEM;
	}

	desc->callback = gxmicro_dma_complete;
	desc->callback_param = gdev;
	dmaengine_submit(desc);

	dma_async_issue_pending(chan);

	gxmicro_write(gdev, SATA_LABL, lbal);
	gxmicro_write(gdev, SATA_LABH, lbah);
	gxmicro_write(gdev, SATA_NSECT, nsect);
	gxmicro_write(gdev, SATA_CMD, cmd);

	return AC_ERR_OK;
}

#define SIM_DELAY	0

static unsigned int gxmicro_issue_cmd_id(struct ata_queued_cmd *qc)
{
	struct ata_device *adev = qc->dev;
	struct ata_port *ap = qc->ap;
	struct gxmicro_sata_dev *gdev = ap->private_data;
	size_t nbytes = 0;

	nbytes = sg_copy_from_buffer(qc->sg, qc->n_elem, adev->id, ATA_ID_DWORD * 4);
	if (!nbytes) {
		ata_port_err(ap, "Failed to issue ATA_CMD_ID_ATA \n");
		return AC_ERR_SYSTEM;
	}

	schedule_delayed_work(&gdev->sim, SIM_DELAY);

	return AC_ERR_OK;
}

static unsigned int gxmicro_issue_cmd_dummy(struct ata_queued_cmd *qc)
{
	struct ata_port *ap = qc->ap;
	struct gxmicro_sata_dev *gdev = ap->private_data;

	schedule_delayed_work(&gdev->sim, SIM_DELAY);

	return AC_ERR_OK;
}

static unsigned int gxmicro_qc_issue(struct ata_queued_cmd *qc)
{
	switch (qc->tf.command) {
	case ATA_CMD_READ:
	case ATA_CMD_READ_EXT:
	case ATA_CMD_WRITE:
	case ATA_CMD_WRITE_EXT:
	case ATA_CMD_PIO_READ:
	case ATA_CMD_PIO_READ_EXT:
	case ATA_CMD_PIO_WRITE:
	case ATA_CMD_PIO_WRITE_EXT:
		return gxmicro_issue_cmd_rw(qc);
	case ATA_CMD_ID_ATA:
		return gxmicro_issue_cmd_id(qc);
	case ATA_CMD_FLUSH:
	case ATA_CMD_FLUSH_EXT:
	case ATA_CMD_DSM:
		return gxmicro_issue_cmd_dummy(qc);
	default:
		ata_port_err(qc->ap, "Can't process command 0x%x", qc->tf.command);
		return AC_ERR_INVALID;
	}
}

static bool gxmicro_qc_fill_rtf(struct ata_queued_cmd *qc)
{
	struct gxmicro_sata_dev *gdev = qc->ap->private_data;
	struct ata_taskfile *tf = &qc->result_tf;
	u8 fsr, ferr;

	fsr = gxmicro_read(gdev, SATA_FIS_SR);
	ferr = gxmicro_read(gdev, SATA_FIS_ERR);

	tf->status = fsr;
	tf->error = ferr;

	return true;
}

static int gxmicro_set_mode(struct ata_link *link, struct ata_device **unused)
{
	struct ata_device *dev;

	ata_for_each_dev(dev, link, ENABLED) {
		dev->pio_mode = XFER_PIO_4;
		dev->dma_mode = XFER_UDMA_6;
		dev->xfer_mode = XFER_UDMA_6;
		dev->xfer_shift = ATA_SHIFT_UDMA;
		dev->flags &= ~ATA_DFLAG_PIO;
	}

	return 0;
}

static unsigned int gxmicro_read_id(struct ata_device *dev,
				struct ata_taskfile *tf, __le16 *id)
{
	struct ata_port *ap = dev->link->ap;
	struct gxmicro_sata_dev *gdev = ap->private_data;
	__le32 *gid = (__le32 *)id;
	int i;

	gxmicro_write(gdev, SATA_CMD, SATA_CMD_ID);

	for (i = 0; i < ATA_ID_DWORD; i++)
		gid[i] = gxmicro_read(gdev, SATA_RAM + i * 4);
#if 0
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 2,
			id, ATA_ID_DWORD * 4, true);
#endif
	return AC_ERR_OK;
}

static int gxmicro_softreset(struct ata_link *link, unsigned int *class,
			unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	struct gxmicro_sata_dev *gdev = ap->private_data;

	dmaengine_terminate_async(gdev->s2mm);
	dmaengine_terminate_async(gdev->mm2s);

	gxmicro_write(gdev, SATA_CMD, SATA_CMD_RST);

	*class = ATA_DEV_ATA;

	return 0;
}

static int gxmicro_scr_reg(unsigned int sc_reg)
{
	switch (sc_reg) {
	case SCR_STATUS:
		return SATA_SSR;
	case SCR_ERROR:
		return SATA_SERR;
	default:
		return -EINVAL;
	}
}

static int gxmicro_scr_read(struct ata_link *link, unsigned int sc_reg, u32 *val)
{
	struct ata_port *ap = link->ap;
	struct gxmicro_sata_dev *gdev = ap->private_data;
	int reg;

	reg = gxmicro_scr_reg(sc_reg);
	if (reg < 0)
		return reg;

	*val = gxmicro_read(gdev, reg);

	return 0;
}

static int gxmicro_scr_write(struct ata_link *link, unsigned int sc_reg, u32 val)
{
	struct ata_port *ap = link->ap;
	struct gxmicro_sata_dev *gdev = ap->private_data;
	int reg;

	reg = gxmicro_scr_reg(sc_reg);
	if (reg < 0)
		return reg;

	gxmicro_write(gdev, reg, val);

	return 0;
}

static int gxmicro_port_start(struct ata_port *ap)
{
	struct ata_host *host = ap->host;
	struct gxmicro_sata_dev *gdev = host->private_data;

	INIT_DELAYED_WORK(&gdev->sim, gxmicro_simulate_complete);

	gxmicro_write(gdev, SATA_CR, SATA_CR_EN);

	ap->private_data = gdev;

	return 0;
}

static void gxmicro_port_stop(struct ata_port *ap)
{
	struct gxmicro_sata_dev *gdev = ap->private_data;

	gxmicro_write(gdev, SATA_CR, SATA_CR_DIS);

	cancel_delayed_work_sync(&gdev->sim);
}

static struct ata_port_operations aops = {
	.qc_defer = ata_std_qc_defer,
	.qc_prep = gxmicro_qc_prep,
	.qc_issue = gxmicro_qc_issue,
	.qc_fill_rtf = gxmicro_qc_fill_rtf,
	.set_mode = gxmicro_set_mode,
	.read_id = gxmicro_read_id,
	.softreset = gxmicro_softreset,
	.error_handler = ata_std_error_handler,
	.sched_eh = ata_std_sched_eh,
	.end_eh = ata_std_end_eh,
	.scr_read = gxmicro_scr_read,
	.scr_write = gxmicro_scr_write,
#if 0
	.port_suspend = gxmicro_port_suspend,
	.port_resume = gxmicro_port_resume,
#endif
	.port_start = gxmicro_port_start,
	.port_stop = gxmicro_port_stop,
};

const struct ata_port_info gxmicro_port_info = {
	.flags = ATA_FLAG_SATA |
		ATA_FLAG_PIO_DMA |
		ATA_FLAG_NO_LOG_PAGE,
	.pio_mask = ATA_PIO4,
	.udma_mask = ATA_UDMA6,
	.port_ops = &aops,
};

static struct scsi_host_template gxmicro_sata_scsi_sht = {
	__ATA_BASE_SHT(KBUILD_MODNAME),
	.can_queue = ATA_DEF_QUEUE,
	.sg_tablesize = SG_ALL,
	.dma_boundary = GENMASK(24, 0),
	.change_queue_depth = ata_scsi_change_queue_depth,
	.tag_alloc_policy = BLK_TAG_ALLOC_RR,
	.slave_configure = ata_scsi_slave_config
};

int gxmicro_ata_init(struct platform_device *pdev, struct gxmicro_sata_dev *gdev)
{
	const struct ata_port_info *ppi[] = { &gxmicro_port_info, NULL };
	struct ata_host *host;
	int ret;

	host = ata_host_alloc_pinfo(&pdev->dev, ppi, 1); /* platform_set_drvdata(pdev, host) */
	if (!host) {
		dev_err(&pdev->dev, "Failed to allocate ATA host\n");
		return -ENOMEM;
	}

	gdev->host = host;
	host->private_data = gdev;

	ret = ata_host_start(host);
	if (ret) {
		dev_err(&pdev->dev, "Failed to start ATA host\n");
		return ret;
	}

	ret = ata_host_register(host, &gxmicro_sata_scsi_sht);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register ATA host\n");
		return ret;
	}

	return 0;
}

void gxmicro_ata_fini(struct platform_device *pdev, struct gxmicro_sata_dev *gdev)
{
	struct ata_host *host = platform_get_drvdata(pdev);

	ata_host_detach(host);
}
