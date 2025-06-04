// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro DMA
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/completion.h>

#include "gxmicro_sata.h"

int gxmicro_dma_init(struct platform_device *pdev, struct gxmicro_sata_dev *gdev)
{
	int ret;

	gdev->mm2s = dma_request_chan(&pdev->dev, "mm2s");
	if (IS_ERR(gdev->mm2s)) {
		dev_err(&pdev->dev, "Failed to request mm2s channel\n");
		return PTR_ERR(gdev->mm2s);
	}

	gdev->s2mm = dma_request_chan(&pdev->dev, "s2mm");
	if (IS_ERR(gdev->mm2s)) {
		dev_err(&pdev->dev, "Failed to request s2mm channel\n");
		ret = PTR_ERR(gdev->mm2s);
		goto err_request_chan;
	}

	return 0;

err_request_chan:
	dma_release_channel(gdev->mm2s);
	return ret;
}

void gxmicro_dma_fini(struct platform_device *pdev, struct gxmicro_sata_dev *gdev)
{
	dmaengine_terminate_sync(gdev->s2mm);
	dma_release_channel(gdev->s2mm);

	dmaengine_terminate_sync(gdev->mm2s);
	dma_release_channel(gdev->mm2s);
}
