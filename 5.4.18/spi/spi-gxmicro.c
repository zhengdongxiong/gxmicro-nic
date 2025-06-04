// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro SPI Controller driver
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Base on:
 * 	spi-xilinx
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/completion.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>

#define SPI_SRR			0x40
#define  SPI_RESET		0x0a
#define SPI_CR			0x60
#define  SPI_CR_LSB_FIRST	BIT(9)
#define  SPI_CR_TRANS_INHIBIT	BIT(8)
#define  SPI_CR_MANUAL_SSELECT	BIT(7)
#define  SPI_CR_RXFIFO_RESET	BIT(6)
#define  SPI_CR_TXFIFO_RESET	BIT(5)
#define  SPI_CR_CPHA		BIT(4)
#define  SPI_CR_CPOL		BIT(3)
#define  SPI_CR_MASTER_MODE	BIT(2)
#define  SPI_CR_ENABLE		BIT(1)
#define  SPI_CR_LOOP		BIT(0)
#define  SPI_CR_MODE		(SPI_CR_CPHA | SPI_CR_CPOL)
#define  SPI_CR_INIT		(SPI_CR_MANUAL_SSELECT | SPI_CR_RXFIFO_RESET | \
				SPI_CR_TXFIFO_RESET | SPI_CR_MASTER_MODE | SPI_CR_ENABLE)
#define SPI_SR			0x64
#define  SPI_SR_MODE_FAULT	BIT(4) /* Mode fault error */
#define  SPI_SR_TX_FULL		BIT(3) /* Transmit FIFO is full */
#define  SPI_SR_TX_EMPTY	BIT(2) /* Transmit FIFO is empty */
#define  SPI_SR_RX_FULL		BIT(1) /* Receive FIFO is full */
#define  SPI_SR_RX_EMPTY	BIT(0) /* Receive FIFO is empty */
#define SPI_DTR			0x68
#define SPI_DRR			0x6c
#define SPI_SSR			0x70
#define  SPI_SSR_INACTIVE	BIT(0)
#define  SPI_SSR_ACTIVE		0x0
#define SPI_GIER		0x1c
#define  SPI_GIER_EN		BIT(31)
#define  SPI_GIER_DIS		0x00
#define SPI_ISR			0x20
#define SPI_IER			0x28
#define  SPI_INTR_TX_HALF_EMPTY	BIT(6) /* TxFIFO is half empty */
#define  SPI_INTR_RX_OVERRUN	BIT(5) /* RxFIFO was overrun */
#define  SPI_INTR_RX_FULL	BIT(4) /* RxFIFO is full */
#define  SPI_INTR_TX_UNDERRUN	BIT(3) /* TxFIFO was underrun */
#define  SPI_INTR_TX_EMPTY	BIT(2) /* TxFIFO is empty */
#define  SPI_INTR_DIS		0x00

struct gxmicro_spi_dev {
	struct spi_bitbang sbit;

	void __iomem *mmio;

	struct completion done;

	u8 *rptr; /* pointer in the Tx buffer */
	const u8 *tptr; /* pointer in the Rx buffer */
};

static inline u32 gxmicro_read(struct gxmicro_spi_dev *gdev, u32 reg)
{
	return ioread32(gdev->mmio + reg);
}

static inline void gxmicro_write(struct gxmicro_spi_dev *gdev, u32 reg, u32 val)
{
	iowrite32(val, gdev->mmio + reg);
}

/* ****************************** Platform ****************************** */

static irqreturn_t gxmicro_spi_irq(int irq, void *data)
{
	struct gxmicro_spi_dev *gdev = data;
	u32 isr;

	/* Get the IPIF interrupts, and clear them immediately */
	isr = gxmicro_read(gdev, SPI_ISR);
	gxmicro_write(gdev, SPI_ISR, isr);
	if (!(isr & SPI_INTR_TX_EMPTY))
		return IRQ_NONE;

	complete(&gdev->done);

	return IRQ_HANDLED;
}

static int gxmicro_plat_init(struct platform_device *pdev)
{
	struct gxmicro_spi_dev *gdev = platform_get_drvdata(pdev);
	int irq;
	int ret;

	gdev->mmio = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(gdev->mmio))
		return PTR_ERR(gdev->mmio);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, gxmicro_spi_irq,
			IRQF_SHARED, KBUILD_MODNAME, gdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq\n");
		return ret;
	}

	return 0;
}

/* ****************************** SPI ****************************** */
#define SPI_XFER_WIDTH		8
#define SPI_BYTES_PER_WORD	(SPI_XFER_WIDTH / BITS_PER_BYTE)
/*
 * write(SPI_SRR, SPI_RESET);
 * udelay(1);
 * do {
 * 	write(SPI_DTR, 0);
 * 	sr = read(SPI_SR);
 * 	buf_size++;
 * } while (!(sr & SPI_SR_TX_FULL));
 */
#define SPI_BUF_SIZE		SZ_256

static int gxmicro_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct gxmicro_spi_dev *gdev = spi_controller_get_devdata(spi->controller);
	u32 cr;

	cr = gxmicro_read(gdev, SPI_CR);

	cr &= ~SPI_CR_MODE;
	if (spi->mode & SPI_CPHA)
		cr |= SPI_CR_CPHA;
	if (spi->mode & SPI_CPOL)
		cr |= SPI_CR_CPOL;

	gxmicro_write(gdev, SPI_CR, cr);

	return 0;
}

static void gxmicro_chipselect(struct spi_device *spi, int cs)
{
	struct gxmicro_spi_dev *gdev = spi_controller_get_devdata(spi->controller);

	if (cs == BITBANG_CS_INACTIVE)
		gxmicro_write(gdev, SPI_SSR, SPI_SSR_INACTIVE);
	else
		gxmicro_write(gdev, SPI_SSR, SPI_SSR_ACTIVE);
}

static void gxmicro_spi_tx(struct gxmicro_spi_dev *gdev)
{
	if (!gdev->tptr) {
		gxmicro_write(gdev, SPI_DTR, 0);
		return ;
	}

	gxmicro_write(gdev, SPI_DTR, *gdev->tptr);
	gdev->tptr += SPI_BYTES_PER_WORD;
}

static void gxmicro_spi_rx(struct gxmicro_spi_dev *gdev)
{
	u32 val = gxmicro_read(gdev, SPI_DRR);

	if (!gdev->rptr)
		return ;

	*gdev->rptr = val;
	gdev->rptr += SPI_BYTES_PER_WORD;
}

static void gxmicro_init_hw(struct gxmicro_spi_dev *gdev)
{
	/* Reset the SPI*/
	gxmicro_write(gdev, SPI_SRR, SPI_RESET);
	udelay(1);

	/*
	 * Enable the transmit empty interrupt, which we use to determine
	 * progress on the transmission.
	 */
	gxmicro_write(gdev, SPI_IER, SPI_INTR_TX_EMPTY);
	/* Disable the global IPIF interrupt */
	gxmicro_write(gdev, SPI_GIER, SPI_GIER_DIS);
	/* Deselect the slave on the SPI bus */
	gxmicro_write(gdev, SPI_SSR, SPI_SSR_INACTIVE);
	/*
	 * Disable the transmitter, enable Manual Slave Select Assertion,
	 * put SPI controller into master mode, and enable it
	 */
	gxmicro_write(gdev, SPI_CR, SPI_CR_INIT);
}

static int gxmicro_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct gxmicro_spi_dev *gdev = spi_controller_get_devdata(spi->controller);
	int remain = t->len / SPI_BYTES_PER_WORD;
	u32 cr = 0, isr = 0;
	int min_words, tx_words, rx_words;
	bool irq = false;
	u32 sr;
	int stalled;

	gdev->tptr = t->tx_buf;
	gdev->rptr = t->rx_buf;

	if (remain > SPI_BUF_SIZE) {
		irq = true;

		/* Inhibit irq to avoid spurious irqs on tx_empty*/
		cr = gxmicro_read(gdev, SPI_CR);
		gxmicro_write(gdev, SPI_CR, cr | SPI_CR_TRANS_INHIBIT);

		/* ACK old irqs (if any) */
		isr = gxmicro_read(gdev, SPI_ISR);
		if (isr)
			gxmicro_write(gdev, SPI_ISR, isr);

		/* Enable the global IPIF interrupt */
		gxmicro_write(gdev, SPI_GIER, SPI_GIER_EN);
		reinit_completion(&gdev->done);
	}

	while (remain) {
		min_words = min(remain, SPI_BUF_SIZE);

		tx_words = min_words;
		while (tx_words--)
			gxmicro_spi_tx(gdev);

		/*
		 * Start the transfer by not inhibiting
		 * the transmitter any longer
		 */
		if (irq) {
			gxmicro_write(gdev, SPI_CR, cr);
			wait_for_completion(&gdev->done);

			/*
			 * A transmit has just completed. Process received data
			 * and check for more data to transmit. Always inhibit
			 * the transmitter while the Isr refills the transmit
			 * register/FIFO, or make sure it is stopped if we're
			 * done.
			 */
			gxmicro_write(gdev, SPI_CR, cr | SPI_CR_TRANS_INHIBIT);
			sr = SPI_SR_TX_EMPTY;
		} else {
			sr = gxmicro_read(gdev, SPI_SR);
		}

		/* Read out all the data from the Rx FIFO */
		rx_words = min_words;
		stalled = 10;
		while (rx_words) {
			if (rx_words == min_words && !(stalled--) &&
			!(sr & SPI_SR_TX_EMPTY) && (sr & SPI_SR_RX_EMPTY)) {
				dev_err(&spi->dev, "Detected stall\n");
				gxmicro_init_hw(gdev);
				return -EIO;
			}

			if ((sr & SPI_SR_TX_EMPTY) && (rx_words > 1)) {
				gxmicro_spi_rx(gdev);
				rx_words--;
				continue;
			}

			sr = gxmicro_read(gdev, SPI_SR);
			if (!(sr & SPI_SR_RX_EMPTY)) {
				gxmicro_spi_rx(gdev);
				rx_words--;
			}
		}

		remain -= min_words;
	}

	if (irq) {
		gxmicro_write(gdev, SPI_GIER, SPI_GIER_DIS);
		gxmicro_write(gdev, SPI_CR, cr);
	}

	return t->len;
}

static struct mtd_partition fpga_flash_part[] = {
	[0] = {
		.name = "Logic",
		.offset = 0,
		.size = SZ_16M,
	},
	[1] = {
		.name = "Reserved",
		.offset = MTDPART_OFS_APPEND,
		.size = SZ_4K,
	},
	[2] = {
		.name = "Free",
		.offset = MTDPART_OFS_APPEND,
		.size = MTDPART_SIZ_FULL
	}
};

#include <linux/spi/flash.h>
static struct flash_platform_data gxmicro_flash_data = {
	.name = "FPGA",
	.parts = fpga_flash_part,
	.nr_parts = ARRAY_SIZE(fpga_flash_part),
	.type = "mt25qu512a",
};

static struct spi_board_info gxmicro_spi_board[] = {
	{
		.modalias = "spi-nor" ,
		.platform_data = &gxmicro_flash_data,
	}
};

static int gxmicro_spi_init(struct platform_device *pdev)
{
	struct gxmicro_spi_dev *gdev = platform_get_drvdata(pdev);
	struct spi_controller *controller = gdev->sbit.master;
	int ret;
	int i;

	controller->mode_bits = SPI_CPOL | SPI_CPHA;
	controller->bus_num = -1;
	controller->num_chipselect = 1;
	controller->bits_per_word_mask = SPI_BPW_MASK(SPI_XFER_WIDTH);

	gdev->sbit.setup_transfer = gxmicro_setup_transfer;
	gdev->sbit.chipselect =	gxmicro_chipselect;
	gdev->sbit.txrx_bufs =	gxmicro_txrx_bufs;

	gxmicro_init_hw(gdev);
	init_completion(&gdev->done);

	ret = spi_bitbang_start(&gdev->sbit);
	if (ret) {
		dev_err(&pdev->dev, "Failed to init spi bitbang\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(gxmicro_spi_board); i++)
		spi_new_device(controller, &gxmicro_spi_board[i]);

	return 0;
}

static void gxmicro_spi_fini(struct platform_device *pdev)
{
	struct gxmicro_spi_dev *gdev = platform_get_drvdata(pdev);

	spi_bitbang_stop(&gdev->sbit);

	gxmicro_write(gdev, SPI_SRR, SPI_RESET);
}

/* ****************************** Platform Init & Fini ****************************** */

static int gxmicro_spi_probe(struct platform_device *pdev)
{
	struct gxmicro_spi_dev *gdev;
	struct spi_controller *controller;
	int ret;

	controller = spi_alloc_master(&pdev->dev, sizeof(struct gxmicro_spi_dev));
	if (!controller)
		return -ENOMEM;

	gdev = spi_controller_get_devdata(controller);
	gdev->sbit.master = controller;
	platform_set_drvdata(pdev, gdev);

	ret = gxmicro_plat_init(pdev);
	if (ret)
		goto err_plat_init;

	ret = gxmicro_spi_init(pdev);
	if (ret)
		goto err_plat_init;

	return 0;

err_plat_init:
	spi_controller_put(controller);
	return ret;
}

static int gxmicro_spi_remove(struct platform_device *pdev)
{
	struct gxmicro_spi_dev *gdev = platform_get_drvdata(pdev);

	gxmicro_spi_fini(pdev);

	spi_controller_put(gdev->sbit.master);

	return 0;
}

static const struct platform_device_id gxmicro_spi_ids[] = {
	{ .name = KBUILD_MODNAME },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(platform, gxmicro_spi_ids);

static struct platform_driver gxmicro_spi_driver = {
	.probe = gxmicro_spi_probe,
	.remove = gxmicro_spi_remove,
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.id_table = gxmicro_spi_ids,
};
module_platform_driver(gxmicro_spi_driver);

MODULE_DESCRIPTION("GXMicro SPI Controller driver");
MODULE_AUTHOR("DongXiong Zheng <zhengdongxiong@gxmicro.cn>");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
