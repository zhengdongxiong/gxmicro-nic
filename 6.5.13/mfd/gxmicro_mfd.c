// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro MFD driver
 * Support for GXMicro Network Interface Card
 * 	Currently supported devices are:
 * 		ETH
 *		SATA
 *		SDMA
 * 		SPI
 * 		I2C
 *		UARTLite
 * 	Currently supported drivers are:
 * 		ETH: gxmicro_eth
 * 		SATA: sata_gxmicro
 * 		SDMA: gxmicro_sdma
 * 		SPI: spi-gxmicro
 * 		I2C: i2c_gxmicro
 * 		UARTLite: gxmicro_ulite
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/mfd/core.h>
#include <linux/property.h>

/* ****************************** PCIe ****************************** */

//#define GXMICRO_MFD_ETH0
//#define GXMICRO_MFD_ETH1
//#define GXMICRO_MFD_ETH2
//#define GXMICRO_MFD_ETH3
#define GXMICRO_MFD_SATA
#define GXMICRO_MFD_SPI
#define GXMICRO_MFD_I2C
#define GXMICRO_MFD_ULITE

#define ETH0_ID		0
#define ETH1_ID		1
#define ETH2_ID		2
#define ETH3_ID		3
#define SATA_ID		0
#define SDMA_ID		0
#define SPI_ID		0
#define I2C_ID		0
#define ULITE_ID	0

enum gxmicro_mfd_bars {
	ETH_BAR,
	SATA_BAR,
	SPI_BAR,
	I2C_BAR,
	ULITE_BAR,
};

enum gxmicro_mfd_msi {
	ETH0_IRQ,
	ETH0_MM2S_IRQ,
	ETH0_S2MM_IRQ,
	ETH1_IRQ,
	ETH1_MM2S_IRQ,
	ETH1_S2MM_IRQ,
	ETH2_IRQ,
	ETH2_MM2S_IRQ,
	ETH2_S2MM_IRQ,
	ETH3_IRQ,
	ETH3_MM2S_IRQ,
	ETH3_S2MM_IRQ,
	SATA_IRQ,
	SDMA_MM2S_IRQ,
	SDMA_S2MM_IRQ,
	SPI_IRQ,
	I2C_IRQ,
	ULITE_IRQ,
	GXMICRO_NR_MSI
};

static int gxmicro_pcie_init(struct pci_dev *pdev)
{
	int ret;

	pci_set_master(pdev);

	ret = pci_enable_device(pdev);
	if (ret) {
		pci_err(pdev, "Failed to enable pci device\n");
		goto err_pci_enable;
		return ret;
	}

	ret = pci_alloc_irq_vectors(pdev, GXMICRO_NR_MSI, GXMICRO_NR_MSI, PCI_IRQ_MSI);
	if (ret < 0) {
		pci_err(pdev, "Failed to alloc msi vectors\n");
		goto err_alloc_irq_vectors;
	}

	return 0;

err_alloc_irq_vectors:
	pci_disable_device(pdev);
err_pci_enable:
	pci_clear_master(pdev);
	return ret;
}

static void gxmicro_pcie_fini(struct pci_dev *pdev)
{
	pci_free_irq_vectors(pdev);

	pci_disable_device(pdev);

	pci_clear_master(pdev);
}

/* ****************************** MFD ETH ****************************** */
#define GXMICRO_ETH		"gxmicro_eth"
#define GXMICRO_ETH_SZ		SZ_128K
#define GXMICRO_ETHN_ST(b, n)	(pci_resource_start(pdev, b) + n * GXMICRO_ETH_SZ)
#define GXMICRO_ETHN_ED(b, n)	(GXMICRO_ETHN_ST(b, n) + GXMICRO_ETH_SZ - 1)

#ifdef GXMICRO_MFD_ETH0
static struct resource gxmicro_eth0_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_eth0_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_eth0_res;

	res[0].start = GXMICRO_ETHN_ST(ETH_BAR, ETH0_ID);
	res[0].end = GXMICRO_ETHN_ED(ETH_BAR, ETH0_ID);

	res[1].start = pci_irq_vector(pdev, ETH0_MM2S_IRQ);
	res[1].end = pci_irq_vector(pdev, ETH0_MM2S_IRQ);
	res[2].start = pci_irq_vector(pdev, ETH0_S2MM_IRQ);
	res[2].end = pci_irq_vector(pdev, ETH0_S2MM_IRQ);
}
#else
static void gxmicro_eth0_init(struct pci_dev *pdev)
{

}
#endif
#ifdef GXMICRO_MFD_ETH1
static struct resource gxmicro_eth1_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_eth1_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_eth1_res;

	res[0].start = GXMICRO_ETHN_ST(ETH_BAR, ETH1_ID);
	res[0].end = GXMICRO_ETHN_ED(ETH_BAR, ETH1_ID);

	res[1].start = pci_irq_vector(pdev, ETH1_MM2S_IRQ);
	res[1].end = pci_irq_vector(pdev, ETH1_MM2S_IRQ);
	res[2].start = pci_irq_vector(pdev, ETH1_S2MM_IRQ);
	res[2].end = pci_irq_vector(pdev, ETH1_S2MM_IRQ);
}
#else
static void gxmicro_eth1_init(struct pci_dev *pdev)
{

}
#endif
#ifdef GXMICRO_MFD_ETH2
static struct resource gxmicro_eth2_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_eth2_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_eth2_res;

	res[0].start = GXMICRO_ETHN_ST(ETH_BAR, ETH2_ID);
	res[0].end = GXMICRO_ETHN_ED(ETH_BAR, ETH2_ID);

	res[1].start = pci_irq_vector(pdev, ETH2_MM2S_IRQ);
	res[1].end = pci_irq_vector(pdev, ETH2_MM2S_IRQ);
	res[2].start = pci_irq_vector(pdev, ETH2_S2MM_IRQ);
	res[2].end = pci_irq_vector(pdev, ETH2_S2MM_IRQ);
}
#else
static void gxmicro_eth2_init(struct pci_dev *pdev)
{

}
#endif
#ifdef GXMICRO_MFD_ETH3
static struct resource gxmicro_eth3_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_eth3_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_eth3_res;

	res[0].start = GXMICRO_ETHN_ST(ETH_BAR, ETH3_ID);
	res[0].end = GXMICRO_ETHN_ED(ETH_BAR, ETH3_ID);

	res[1].start = pci_irq_vector(pdev, ETH3_MM2S_IRQ);
	res[1].end = pci_irq_vector(pdev, ETH3_MM2S_IRQ);
	res[2].start = pci_irq_vector(pdev, ETH3_S2MM_IRQ);
	res[2].end = pci_irq_vector(pdev, ETH3_S2MM_IRQ);
}
#else
static void gxmicro_eth3_init(struct pci_dev *pdev)
{

}
#endif

/* ****************************** MFD AHCI ****************************** */
#define GXMICRO_SATA		"sata_gxmicro"
#define GXMICRO_SDMA		"gxmicro_sdma"
#define GXMICRO_SATA_SZ		SZ_256K
#define GXMICRO_SATA_ST(b)	(pci_resource_start(pdev, b))
#define GXMICRO_SATA_ED(b)	(GXMICRO_SATA_ST(b) + GXMICRO_SATA_SZ - 1)
#define GXMICRO_SDMA_ST(b)	(pci_resource_start(pdev, b) + GXMICRO_SATA_SZ)
#define GXMICRO_SDMA_ED(b)	(GXMICRO_SDMA_ST(b) + GXMICRO_SATA_SZ - 1)
#ifdef GXMICRO_MFD_SATA

static struct resource gxmicro_sata_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
};

static struct resource gxmicro_sdma_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_sata_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_sata_res;

	res[0].start = GXMICRO_SATA_ST(SATA_BAR);
	res[0].end = GXMICRO_SATA_ED(SATA_BAR);

	res = gxmicro_sdma_res;
	res[0].start = GXMICRO_SDMA_ST(SATA_BAR);
	res[0].end = GXMICRO_SDMA_ED(SATA_BAR);
	res[1].start = pci_irq_vector(pdev, SDMA_MM2S_IRQ);
	res[1].end = pci_irq_vector(pdev, SDMA_MM2S_IRQ);
	res[2].start = pci_irq_vector(pdev, SDMA_S2MM_IRQ);
	res[2].end = pci_irq_vector(pdev, SDMA_S2MM_IRQ);
}
#else
static void gxmicro_sata_init(struct pci_dev *pdev)
{

}
#endif

/* ****************************** MFD SPI ****************************** */
#define GXMICRO_SPI	"spi_gxmicro"
#ifdef GXMICRO_MFD_SPI

static struct resource gxmicro_spi_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_spi_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_spi_res;

	res[0].start = pci_resource_start(pdev, SPI_BAR);
	res[0].end = pci_resource_end(pdev, SPI_BAR);

	res[1].start = pci_irq_vector(pdev, SPI_IRQ);
	res[1].end = pci_irq_vector(pdev, SPI_IRQ);
}
#else
static void gxmicro_spi_init(struct pci_dev *pdev)
{

}
#endif

/* ****************************** MFD I2C ****************************** */
#define GXMICRO_I2C	"i2c_gxmicro"
#ifdef GXMICRO_MFD_I2C

static struct resource gxmicro_i2c_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_i2c_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_i2c_res;

	res[0].start = pci_resource_start(pdev, I2C_BAR);
	res[0].end = pci_resource_end(pdev, I2C_BAR);

	res[1].start = pci_irq_vector(pdev, I2C_IRQ);
	res[1].end = pci_irq_vector(pdev, I2C_IRQ);
}
#else
static void gxmicro_i2c_init(struct pci_dev *pdev)
{

}
#endif

/* ****************************** MFD UARTLite ****************************** */
#define GXMICRO_ULITE	"gxmicro_ulite"
#ifdef GXMICRO_MFD_ULITE

static struct resource gxmicro_ulite_res[] = {
	[0] = {
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.flags = IORESOURCE_IRQ,
	},
};

static void gxmicro_ulite_init(struct pci_dev *pdev)
{
	struct resource *res = gxmicro_ulite_res;

	res[0].start = pci_resource_start(pdev, ULITE_BAR);
	res[0].end = pci_resource_end(pdev, ULITE_BAR);

	res[1].start = pci_irq_vector(pdev, ULITE_IRQ);
	res[1].end = pci_irq_vector(pdev, ULITE_IRQ);
}
#else
static void gxmicro_ulite_init(struct pci_dev *pdev)
{

}
#endif

/* ****************************** MFD ****************************** */

#define GXMICRO_MFD_CELLS(_name, _id, _resource) \
	{ \
		.name = _name, \
		.id = _id, \
		.num_resources = ARRAY_SIZE(_resource), \
		.resources = _resource, \
	}

static struct mfd_cell gxmicro_mfd_cells[] = {
#ifdef GXMICRO_MFD_ETH0
	GXMICRO_MFD_CELLS(GXMICRO_ETH, ETH0_ID, gxmicro_eth0_res),
#endif
#ifdef GXMICRO_MFD_ETH1
	GXMICRO_MFD_CELLS(GXMICRO_ETH, ETH1_ID, gxmicro_eth1_res),
#endif
#ifdef GXMICRO_MFD_ETH2
	GXMICRO_MFD_CELLS(GXMICRO_ETH, ETH2_ID, gxmicro_eth2_res),
#endif
#ifdef GXMICRO_MFD_ETH3
	GXMICRO_MFD_CELLS(GXMICRO_ETH, ETH3_ID, gxmicro_eth3_res),
#endif
#ifdef GXMICRO_MFD_SATA
	GXMICRO_MFD_CELLS(GXMICRO_SATA, SATA_ID, gxmicro_sata_res),
	GXMICRO_MFD_CELLS(GXMICRO_SDMA, SDMA_ID, gxmicro_sdma_res),
#endif
#ifdef GXMICRO_MFD_SPI
	GXMICRO_MFD_CELLS(GXMICRO_SPI, SPI_ID, gxmicro_spi_res),
#endif
#ifdef GXMICRO_MFD_I2C
	GXMICRO_MFD_CELLS(GXMICRO_I2C, I2C_ID, gxmicro_i2c_res),
#endif
#ifdef GXMICRO_MFD_ULITE
	GXMICRO_MFD_CELLS(GXMICRO_ULITE, ULITE_ID, gxmicro_ulite_res),
#endif
};

static int gxmicro_mfd_init(struct pci_dev *pdev)
{
	int ret;

	ret = devm_mfd_add_devices(&pdev->dev, 0, gxmicro_mfd_cells,
		ARRAY_SIZE(gxmicro_mfd_cells), NULL, 0, NULL);
	if (ret) {
		pci_err(pdev, "Failed to register mfd devices\n");
		return ret;
	}

	return 0;
}

/* ****************************** PCIe Probe & Remove ****************************** */

static int gxmicro_mfd_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret;

	ret = gxmicro_pcie_init(pdev);
	if (ret)
		return ret;

	gxmicro_eth0_init(pdev);

	gxmicro_eth1_init(pdev);

	gxmicro_eth2_init(pdev);

	gxmicro_eth3_init(pdev);

	gxmicro_sata_init(pdev);

	gxmicro_spi_init(pdev);

	gxmicro_i2c_init(pdev);

	gxmicro_ulite_init(pdev);

	ret = gxmicro_mfd_init(pdev);
	if (ret)
		goto err_mfd_init;

	return 0;

err_mfd_init:
	gxmicro_pcie_fini(pdev);
	return ret;
}

static void gxmicro_mfd_remove(struct pci_dev *pdev)
{
	gxmicro_pcie_fini(pdev);
}

#define GXMICRO_VENDOR_ID    	0x10ee
#define GXMICRO_DEVICE_ID    	0x7038

static const struct pci_device_id gxmicro_mfd_ids[] = {
	{ PCI_DEVICE(GXMICRO_VENDOR_ID, GXMICRO_DEVICE_ID) },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(pci, gxmicro_mfd_ids);

static struct pci_driver gxmicro_mfd_drv = {
	.name = KBUILD_MODNAME,
	.id_table = gxmicro_mfd_ids,
	.probe = gxmicro_mfd_probe,
	.remove = gxmicro_mfd_remove,
};
module_pci_driver(gxmicro_mfd_drv);

MODULE_DESCRIPTION("GXMicro MFD driver");
MODULE_AUTHOR("Dongxiong Zheng <zhengdongxiong@gxmicro.cn>");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL v2");
