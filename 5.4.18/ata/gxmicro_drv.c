// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro SATA Controller driver
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/kernel.h>
#include <linux/module.h>

#include "gxmicro_sata.h"

static int gxmicro_plat_init(struct platform_device *pdev, struct gxmicro_sata_dev *gdev)
{
	int ret;

	gdev->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gdev->mmio))
		return PTR_ERR(gdev->mmio);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(&pdev->dev, "No suitable DMA available\n");
		return ret;
	}

	return 0;
}

static int gxmicro_sata_probe(struct platform_device *pdev)
{
	struct gxmicro_sata_dev *gdev;
	int ret;

	gdev = devm_kzalloc(&pdev->dev, sizeof(struct gxmicro_sata_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	ret = gxmicro_plat_init(pdev, gdev);
	if (ret)
		return ret;

	ret = gxmicro_dma_init(pdev, gdev);
	if (ret)
		return ret;

	ret = gxmicro_ata_init(pdev, gdev);
	if (ret)
		goto err_ata_init;

	return 0;

err_ata_init:
	gxmicro_dma_fini(pdev, gdev);
	return ret;
}

static int gxmicro_sata_remove(struct platform_device *pdev)
{
	struct ata_host *host = platform_get_drvdata(pdev);
	struct gxmicro_sata_dev *gdev = host->private_data;

	gxmicro_ata_fini(pdev, gdev);

	gxmicro_dma_fini(pdev, gdev);

	return 0;
}

static const struct platform_device_id gxmicro_sata_ids[] = {
	{ .name = KBUILD_MODNAME },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(platform, gxmicro_sata_ids);

static struct platform_driver gxmicro_sata_driver = {
	.probe = gxmicro_sata_probe,
	.remove = gxmicro_sata_remove,
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.id_table = gxmicro_sata_ids,
};
module_platform_driver(gxmicro_sata_driver);

MODULE_DESCRIPTION("GXMicro SATA Controller driver");
MODULE_AUTHOR("DongXiong Zheng <zhengdongxiong@gxmicro.cn>");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
