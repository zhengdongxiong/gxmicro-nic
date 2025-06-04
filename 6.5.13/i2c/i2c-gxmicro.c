// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro I2C Controller driver
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Base on:
 * 	i2c-xiic
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/io.h>

#define I2C_GIER		0x1c /* Device Global Interrupt Enable Register */
#define  I2C_GIER_EN		BIT(31)
#define I2C_ISR			0x20 /* Interrupt Status Register */
#define I2C_IER			0x28 /* Interrupt Enable Register */
#define  I2C_INTR_TX_HALF	BIT(7) /* 1 = TX FIFO half empty */
#define  I2C_INTR_NAAS		BIT(6) /* 1 = not addr as slave */
#define  I2C_INTR_AAS		BIT(5) /* 1 = when addr as slave */
#define  I2C_INTR_BNB		BIT(4) /* 1 = Bus not busy */
#define  I2C_INTR_RX_FULL	BIT(3) /* 1=Rx FIFO/reg=OCY level */
#define  I2C_INTR_TX_EMPTY	BIT(2) /* 1 = Tx FIFO/reg empty */
#define  I2C_INTR_TX_ERROR	BIT(1) /* 1=Tx error/msg complete */
#define  I2C_INTR_ARB_LOST	BIT(0) /* 1 = arbitration lost */
#define I2C_SRR			0x40 /* Reset Register */
#define  I2C_RESET		0xa
#define I2C_CR			0x100 /* Control Register */
#define  I2C_CR_GC_EN		BIT(6) /* Gen Call enabled = 1 */
#define  I2C_CR_RSTA		BIT(5) /* Repeated start = 1 */
#define  I2C_CR_TXAK		BIT(4) /* Tx Ack. NO ack = 1 */
#define  I2C_CR_TX		BIT(3) /* Dir of tx. Txing=1 */
#define  I2C_CR_MSMS		BIT(2) /* Master starts Txing = 1 */
#define  I2C_CR_TFO_RESET	BIT(1) /* Transmit FIFO reset = 1 */
#define  I2C_CR_EN		BIT(0) /* Device enable = 1 */
#define I2C_SR			0x104 /* Status Register */
#define  I2C_SR_TFO_EMPTY	BIT(7) /* 1 = Tx FIFO empty */
#define  I2C_SR_RFO_EMPTY	BIT(6) /* 1 = Rx FIFO empty */
#define  I2C_SR_RFO_FULL	BIT(5) /* 1 = Rx FIFO full */
#define  I2C_SR_TFO_FULL	BIT(4) /* 1 = Tx FIFO full */
#define  I2C_SR_SRW		BIT(3) /* 1 = Dir: mstr <-- slave */
#define  I2C_SR_BB		BIT(2) /* 1 = bus is busy */
#define  I2C_SR_AAS		BIT(1) /* 1 = when addr as slave */
#define  I2C_SR_ABGC		BIT(0) /* 1 = a mstr issued a GC */
#define I2C_DTR			0x108 /* Data Tx Register */
#define  I2C_DTR_STOP		BIT(9)
#define  I2C_DTR_START		BIT(8)
#define I2C_DRR			0x10c /* Data Rx Register */
#define I2C_ADR			0x110 /* Address Register */
#define I2C_TFO			0x114 /* Tx FIFO Occupancy */
#define I2C_RFO			0x118 /* Rx FIFO Occupancy */
#define I2C_TBA			0x11c /* 10 Bit Address Register */
#define I2C_RFD			0x120 /* Rx FIFO Depth Register */
#define I2C_GPO			0x124 /* Output Register */
#define I2C_TSUSTA		0x128 /* TSUSTA Register */
#define I2C_TSUSTO		0x12c /* TSUSTO Register */
#define I2C_THDSTA		0x130 /* THDSTA Register */
#define I2C_TSUDAT		0x134 /* TSUDAT Register */
#define I2C_TBUF		0x138 /* THDDAT Register */

#define I2C_FIFO_DEPTH		16 /* FIFO depth */
#define GXMICRO_I2C_TIMEOUT	(msecs_to_jiffies(1000))
#define I2CMSG_RX_SPACE(gdev)	((gdev)->rx_msg->len - (gdev)->rx_pos)
#define I2CMSG_TX_SPACE(gdev)	((gdev)->tx_msg->len - (gdev)->tx_pos)

enum gxmicro_i2c_state {
	I2C_DONE,
	I2C_ERROR,
	I2C_START
};

struct gxmicro_i2c_dev {
	void __iomem *mmio;

	struct i2c_adapter adap;
	struct completion done;
	struct mutex lock;

	struct i2c_msg *rx_msg;
	struct i2c_msg *tx_msg;

	unsigned int nmsgs;
	unsigned int rx_pos;
	unsigned int tx_pos;

	enum gxmicro_i2c_state state;
};

static inline u32 gxmicro_read(struct gxmicro_i2c_dev *gdev, u32 reg)
{
	return ioread32(gdev->mmio + reg);
}

static inline void gxmicro_write(struct gxmicro_i2c_dev *gdev, u32 reg, u32 val)
{
	iowrite32(val, gdev->mmio + reg);
}

static inline void gxmicro_reset(struct gxmicro_i2c_dev *gdev)
{
	gxmicro_write(gdev, I2C_SRR, I2C_RESET);
	udelay(1);
}

static inline void gxmicro_irq_dis(struct gxmicro_i2c_dev *gdev, u32 irq)
{
	u32 ier = gxmicro_read(gdev, I2C_IER);
	gxmicro_write(gdev, I2C_IER, ier & ~irq);
}

static inline void gxmicro_irq_clr_en(struct gxmicro_i2c_dev *gdev, u32 irq)
{
	u32 isr = gxmicro_read(gdev, I2C_ISR);
	u32 ier = gxmicro_read(gdev, I2C_IER);

	gxmicro_write(gdev, I2C_ISR, irq & isr);
	gxmicro_write(gdev, I2C_IER, irq | ier);
}

static int gxmicro_clear_rfo(struct gxmicro_i2c_dev *gdev)
{
	unsigned long timeout = jiffies + GXMICRO_I2C_TIMEOUT;
	u32 sr;

	do {
		sr = gxmicro_read(gdev, I2C_SR);
		if (sr & I2C_SR_RFO_EMPTY)
			return 0;

		gxmicro_read(gdev, I2C_DRR);

	} while (time_before(jiffies, timeout));

	dev_err(gdev->adap.dev.parent, "Failed to clear Rx fifo\n");
	return -ETIMEDOUT;
}

static void gxmicro_read_rx(struct gxmicro_i2c_dev *gdev)
{
	int i;
	u8 bytes;

	bytes = gxmicro_read(gdev, I2C_RFO) + 1;

	if (bytes > I2CMSG_RX_SPACE(gdev))
		bytes = I2CMSG_RX_SPACE(gdev);

	/* Read the fifo */
	for (i = 0; i < bytes; i++)
		gdev->rx_msg->buf[gdev->rx_pos++] = gxmicro_read(gdev, I2C_DRR);

	/* Receive remaining bytes if less than fifo depth */
	bytes = min_t(u8, I2CMSG_RX_SPACE(gdev), I2C_FIFO_DEPTH);
	bytes--;
	gxmicro_write(gdev, I2C_RFD, bytes);
}

static inline int gxmicro_tx_fifo_space(struct gxmicro_i2c_dev *gdev)
{
	return I2C_FIFO_DEPTH - gxmicro_read(gdev, I2C_TFO) - 1;
}

static void gxmicro_write_tx(struct gxmicro_i2c_dev *gdev)
{
	u8 fifo_space = gxmicro_tx_fifo_space(gdev);
	int len = I2CMSG_TX_SPACE(gdev);

	len = (len > fifo_space) ? fifo_space : len;

	while (len--) {
		u32 data = gdev->tx_msg->buf[gdev->tx_pos++];

		if (!I2CMSG_TX_SPACE(gdev) && gdev->nmsgs == 1)
			/* last message in transfer -> STOP */
			data |= I2C_DTR_STOP;
		gxmicro_write(gdev, I2C_DTR, data);
	}
}

static void gxmicro_start_recv(struct gxmicro_i2c_dev *gdev)
{
	struct i2c_msg *msg = gdev->rx_msg = gdev->tx_msg;
	u16 val;
	u8 bytes;

	/* Clear and enable Rx full interrupt. */
	gxmicro_irq_dis(gdev, I2C_INTR_TX_EMPTY | I2C_INTR_TX_HALF);

	/* Clear and enable Rx full interrupt. */
	gxmicro_irq_clr_en(gdev, I2C_INTR_RX_FULL | I2C_INTR_TX_ERROR);

	/*
	* We want to get all but last byte, because the TX_ERROR IRQ
	* is used to indicate error ACK on the address, and
	* negative ack on the last received byte, so to not mix
	* them receive all but last.
	* In the case where there is only one byte to receive
	* we can check if ERROR and RX full is set at the same time
	*/
	bytes = min_t(u8, msg->len, I2C_FIFO_DEPTH);
	if (msg->len > 0)
		bytes--;
	gxmicro_write(gdev, I2C_RFD, bytes);

	/* write the address */
	gxmicro_write(gdev, I2C_DTR,
		i2c_8bit_addr_from_msg(msg) | I2C_DTR_START);

	/* If last message, include dynamic stop bit with length */
	val = (gdev->nmsgs == 1) ? I2C_DTR_STOP : 0;
	val |= msg->len;

	gxmicro_write(gdev, I2C_DTR, val);

	gxmicro_irq_clr_en(gdev, I2C_INTR_BNB);

	if (gdev->nmsgs == 1)
		/* very last, enable bus not busy as well */
		gxmicro_irq_clr_en(gdev, I2C_INTR_BNB);

	/* the message is tx:ed */
	gdev->tx_pos = msg->len;

	/* Enable interrupts */
	gxmicro_write(gdev, I2C_GIER, I2C_GIER_EN);
}

static void gxmicro_start_send(struct gxmicro_i2c_dev *gdev)
{
	struct i2c_msg *msg = gdev->tx_msg;
	u32 data;

	/* write the address */
	data = i2c_8bit_addr_from_msg(msg) | I2C_DTR_START;

	if (gdev->nmsgs == 1 && msg->len == 0)
		/* no data and last message -> add STOP */
		data |= I2C_DTR_STOP;

	gxmicro_write(gdev, I2C_DTR, data);

	/* Clear any pending Tx empty, Tx Error and then enable them */
	gxmicro_irq_clr_en(gdev, I2C_INTR_TX_EMPTY |
			I2C_INTR_TX_ERROR | I2C_INTR_BNB |
			((gdev->nmsgs > 1 || I2CMSG_TX_SPACE(gdev)) ?
			I2C_INTR_TX_HALF : 0));

	gxmicro_write_tx(gdev);
}

static void __gxmicro_start_xfer(struct gxmicro_i2c_dev *gdev)
{
	if (!gdev->tx_msg)
		return ;

	gdev->rx_pos = 0;
	gdev->tx_pos = 0;
	gdev->state = I2C_START;
	if (gdev->tx_msg->flags & I2C_M_RD)
		/* we dont date putting several reads in the FIFO */
		gxmicro_start_recv(gdev);
	else
		gxmicro_start_send(gdev);
}

/* ****************************** Platform ****************************** */

static int gxmicro_reinit(struct gxmicro_i2c_dev *gdev)
{
	int ret;

	gxmicro_reset(gdev);

	gxmicro_write(gdev, I2C_RFD, I2C_FIFO_DEPTH - 1);
	gxmicro_write(gdev, I2C_CR, I2C_CR_TFO_RESET);
	gxmicro_write(gdev, I2C_CR, I2C_CR_EN);

	ret = gxmicro_clear_rfo(gdev);
	if (ret)
		return ret;

	gxmicro_write(gdev, I2C_GIER, I2C_GIER_EN);
	gxmicro_irq_clr_en(gdev, I2C_INTR_ARB_LOST);

	return 0;
}

static void gxmicro_deinit(struct gxmicro_i2c_dev *gdev)
{
	u32 cr;

	gxmicro_reset(gdev);

	cr = gxmicro_read(gdev, I2C_CR);
	gxmicro_write(gdev, I2C_CR, cr & ~I2C_CR_EN);
}

static irqreturn_t gxmicro_i2c_irq(int irq, void *data)
{
	struct gxmicro_i2c_dev *gdev = data;
	u32 pend, isr, ier, clr = 0;
	bool wakeup = false, xfer_restart = false;
	enum gxmicro_i2c_state state = I2C_DONE;
	int ret;

	/*
	 * Get the interrupt Status from the IPIF. There is no clearing of
	 * interrupts in the IPIF. Interrupts must be cleared at the source.
	 * To find which interrupts are pending; AND interrupts pending with
	 * interrupts masked.
	 */
	mutex_lock(&gdev->lock);
	isr = gxmicro_read(gdev, I2C_ISR);
	ier = gxmicro_read(gdev, I2C_IER);
	pend = isr & ier;

	/* Service requesting interrupt */
	if ((pend & I2C_INTR_ARB_LOST) ||
	    ((pend & I2C_INTR_TX_ERROR) &&
	    !(pend & I2C_INTR_RX_FULL))) {
		/*
		 * bus arbritration lost, or...
		 * Transmit error _OR_ RX completed
		 * if this happens when RX_FULL is not set
		 * this is probably a TX error
		 */
		ret = gxmicro_reinit(gdev);
		if (ret)
			dev_err(gdev->adap.dev.parent, "Failed to reinit I2C\n");

		if (gdev->rx_msg || gdev->tx_msg) {
			wakeup = true;
			state = I2C_ERROR;
		}

		/* don't try to handle other events */
		goto err_i2c_irq;
	}

	if (pend & I2C_INTR_RX_FULL) {
		clr |= I2C_INTR_RX_FULL;
		if (!gdev->rx_msg) {
			dev_err(gdev->adap.dev.parent, "Unexpected RX IRQ\n");
			gxmicro_clear_rfo(gdev);
			goto err_i2c_irq;
		}

		gxmicro_read_rx(gdev);
		if (I2CMSG_RX_SPACE(gdev) == 0) {
			gdev->rx_msg = NULL;

			clr |= (isr & I2C_INTR_TX_ERROR);

			if (gdev->nmsgs > 1) {
				gdev->nmsgs--;
				gdev->tx_msg++;
				xfer_restart = true;
			}
		}
	}

	if (pend & (I2C_INTR_TX_EMPTY | I2C_INTR_TX_HALF)) {
		clr |= (pend & (I2C_INTR_TX_EMPTY | I2C_INTR_TX_HALF));
		if (!gdev->tx_msg) {
			dev_err(gdev->adap.dev.parent, "Unexpected TX IRQ\n");
			goto err_i2c_irq;
		}

		gxmicro_write_tx(gdev);
		/* current message sent and there is space in the fifo */
		if (!I2CMSG_TX_SPACE(gdev) && gxmicro_tx_fifo_space(gdev) >= 2) {
			if (gdev->nmsgs > 1) {
				gdev->nmsgs--;
				gdev->tx_msg++;
				xfer_restart = true;
			} else {
				gxmicro_irq_dis(gdev, I2C_INTR_TX_HALF);
			}
		} else if (!I2CMSG_TX_SPACE(gdev) && (gdev->nmsgs == 1)) {
			/*
			 * current frame is sent and is last,
			 * make sure to disable tx half
			 */
			gxmicro_irq_dis(gdev, I2C_INTR_TX_HALF);
		}
	}

	if (pend & I2C_INTR_BNB) {
		/* IIC bus has transitioned to not busy */
		clr |= I2C_INTR_BNB;

		gxmicro_irq_dis(gdev, I2C_INTR_BNB);

		if (!gdev->tx_msg)
			goto err_i2c_irq;

		wakeup = true;

		if (gdev->nmsgs == 1 && !gdev->rx_msg &&
			I2CMSG_TX_SPACE(gdev) == 0)
			state = I2C_DONE;
		else
			state = I2C_ERROR;
	}


err_i2c_irq:
	gxmicro_write(gdev, I2C_ISR, clr);

	if (xfer_restart)
		__gxmicro_start_xfer(gdev);
	if (wakeup) {
		gdev->tx_msg = NULL;
		gdev->rx_msg = NULL;
		gdev->nmsgs = 0;
		gdev->state = state;
		complete(&gdev->done);
	}

	mutex_unlock(&gdev->lock);
	return IRQ_HANDLED;
}

static int gxmicro_plat_init(struct platform_device *pdev)
{
	struct gxmicro_i2c_dev *gdev = platform_get_drvdata(pdev);
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

	ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, gxmicro_i2c_irq,
				IRQF_ONESHOT | IRQF_SHARED, KBUILD_MODNAME, gdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq\n");
		return ret;
	}

	return 0;
}

/* ****************************** I2C ****************************** */

static int gxmicro_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct gxmicro_i2c_dev *gdev = i2c_get_adapdata(adap);
	int ret = 0;

	mutex_lock(&gdev->lock);

	if (gdev->tx_msg || gdev->rx_msg) {
		ret = -EBUSY;
		goto err_xfer;
	}

	gdev->tx_msg = msgs;
	gdev->rx_msg = NULL;
	gdev->nmsgs = num;
	init_completion(&gdev->done);

	ret = gxmicro_reinit(gdev);
	if (ret)
		goto err_xfer;

	__gxmicro_start_xfer(gdev);

	mutex_unlock(&gdev->lock);

	ret = wait_for_completion_timeout(&gdev->done, GXMICRO_I2C_TIMEOUT);
	mutex_lock(&gdev->lock);
	if (ret == 0) { /* Timeout */
		gdev->tx_msg = NULL;
		gdev->rx_msg = NULL;
		gdev->nmsgs = 0;
		ret = -ETIMEDOUT;
		dev_err(adap->dev.parent, "Wait for I2C Xfer timeout\n");
	} else {
		ret = (gdev->state == I2C_DONE) ? num : -EIO;
	}

err_xfer:
	mutex_unlock(&gdev->lock);
	return ret;
}

static u32 gxmicro_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm gxmicro_i2c_algorithm = {
	.master_xfer = gxmicro_xfer,
	.functionality = gxmicro_func,
};

static const struct i2c_adapter_quirks gxmicro_i2c_quirks = {
	.max_read_len = 255,
};

#include <linux/platform_data/ina2xx.h>
static struct ina2xx_platform_data ina226_shunt = {
	.shunt_uohms = 5000,
};

static struct i2c_board_info gxmicro_i2c_board[] = {
	{ I2C_BOARD_INFO("jc42", 0x18) },
	{ I2C_BOARD_INFO("ina226", 0x41), .platform_data = &ina226_shunt },
};

static int gxmicro_i2c_init(struct platform_device *pdev)
{
	struct gxmicro_i2c_dev *gdev = platform_get_drvdata(pdev);
	struct i2c_adapter *adap = &gdev->adap;
	int ret;
	int i;

	ret = gxmicro_reinit(gdev);
	if (ret)
		return ret;

	mutex_init(&gdev->lock);
	adap->dev.parent = &pdev->dev;
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->algo = &gxmicro_i2c_algorithm;
	adap->quirks = &gxmicro_i2c_quirks;
	i2c_set_adapdata(adap, gdev);
	snprintf(adap->name, sizeof(adap->name), KBUILD_MODNAME);

	ret = i2c_add_adapter(adap);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add i2c adapter\n");
		goto err_add_adapter;
	}

	for (i = 0; i < ARRAY_SIZE(gxmicro_i2c_board); i++)
		i2c_new_client_device(adap, &gxmicro_i2c_board[i]);

	return 0;

err_add_adapter:
	gxmicro_deinit(gdev);
	return ret;
}

static void gxmicro_i2c_fini(struct platform_device *pdev)
{
	struct gxmicro_i2c_dev *gdev = platform_get_drvdata(pdev);

	i2c_del_adapter(&gdev->adap);

	gxmicro_deinit(gdev);
}

/* ****************************** Platform Init & Fini ****************************** */

static int gxmicro_i2c_probe(struct platform_device *pdev)
{
	struct gxmicro_i2c_dev *gdev;
	int ret;

	gdev = devm_kzalloc(&pdev->dev, sizeof(struct gxmicro_i2c_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, gdev);

	ret = gxmicro_plat_init(pdev);
	if (ret)
		return ret;

	ret = gxmicro_i2c_init(pdev);
	if (ret)
		return ret;

	return 0;
}

static int gxmicro_i2c_remove(struct platform_device *pdev)
{
	gxmicro_i2c_fini(pdev);

	return 0;
}

static const struct platform_device_id gxmicro_i2c_ids[] = {
	{ .name = KBUILD_MODNAME },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(platform, gxmicro_i2c_ids);

static struct platform_driver gxmicro_i2c_driver = {
	.probe = gxmicro_i2c_probe,
	.remove = gxmicro_i2c_remove,
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.id_table = gxmicro_i2c_ids,
};
module_platform_driver(gxmicro_i2c_driver);

MODULE_DESCRIPTION("GXMicro I2C Controller driver");
MODULE_AUTHOR("DongXiong Zheng <zhengdongxiong@gxmicro.cn>");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
