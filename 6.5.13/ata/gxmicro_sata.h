/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXMicro SATA
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#ifndef __GXMICRO_SATA_H__
#define __GXMICRO_SATA_H__

#include <linux/platform_device.h>
#include <linux/dmaengine.h>
#include <linux/libata.h>

#define SATA_CR			0x000
#define  SATA_CR_EN		BIT(26)
#define  SATA_CR_RST		BIT(0)
#define  SATA_CR_DIS		0x0
#define SATA_CMD		0x004
#define  SATA_CMD_EN		BIT(31)
#define  SATA_CMD_RD(nsect)	(SATA_CMD_EN | BIT(28) | (nsect))
#define  SATA_CMD_WR(nsect)	(SATA_CMD_EN | BIT(27) | (nsect))
#define  SATA_CMD_ID		(SATA_CMD_EN | BIT(24))
#define  SATA_CMD_RST		(SATA_CMD_EN | BIT(23))
#define SATA_SR			0x00c
#define  SATA_SR_DONE		GENMASK(1, 0)
#define  SATA_SR_CLR		0x0
#define SATA_DIAG		0x014
#define  SATA_DIAG_DONE		BIT(0)
#define SATA_ASR		0x040
#define  SATA_ASR_DONE		BIT(13)
#define SATA_FIS_SR		0x108
#define SATA_FIS_ERR		0x10c
#define SATA_SSR		0x110
#define SATA_SERR		0x114
#define SATA_LABL		0x10000
#define SATA_LABH		0x10004
#define SATA_NSECT		0x10008
#define SATA_RAM		0x20000

#define SATA_NOOP		0
#define SATA_FIS(f24, f16, f8, f0) \
				(((f24) << 24) | ((f16) << 16) | ((f8) << 8) | (f0))

struct gxmicro_sata_dev {
	void __iomem *mmio;

	struct dma_chan *mm2s;
	struct dma_chan *s2mm;

	struct ata_host *host;

	struct delayed_work sim;
};
#define to_gxmicro_sata_dev(w) \
		container_of(to_delayed_work(w), struct gxmicro_sata_dev, sim)

static inline u32 gxmicro_read(struct gxmicro_sata_dev *gdev, u32 reg)
{
	return ioread32(gdev->mmio + reg);
}

static inline void gxmicro_write(struct gxmicro_sata_dev *gdev, u32 reg, u32 val)
{
	iowrite32(val, gdev->mmio + reg);
}

int gxmicro_dma_init(struct platform_device *pdev, struct gxmicro_sata_dev *gdev);
void gxmicro_dma_fini(struct platform_device *pdev, struct gxmicro_sata_dev *gdev);

int gxmicro_ata_init(struct platform_device *pdev, struct gxmicro_sata_dev *gdev);
void gxmicro_ata_fini(struct platform_device *pdev, struct gxmicro_sata_dev *gdev);

#endif /* __GXMICRO_SATA_H__ */
