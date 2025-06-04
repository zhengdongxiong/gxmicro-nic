// SPDX-License-Identifier: GPL-2.0
/*
 * GXMicro UARTLite Controller driver
 *
 * Copyright (C) 2024 GXMicro (ShangHai) Corp.
 *
 * Author:
 * 	DongXiong Zheng <zhengdongxiong@gxmicro.cn>
 * Base on:
 * 	uartlite.c
 */
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/console.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define ULITE_RX		0x00
#define ULITE_TX		0x04
#define ULITE_SR		0x08
#define  ULITE_SR_RXVALID	BIT(0)
#define  ULITE_SR_RXFULL	BIT(1)
#define  ULITE_SR_TXEMPTY	BIT(2)
#define  ULITE_SR_TXFULL	BIT(3)
#define  ULITE_SR_IE		BIT(4)
#define  ULITE_SR_OVERRUN	BIT(5)
#define  ULITE_SR_FRAME		BIT(6)
#define  ULITE_SR_PARITY	BIT(7)
#define ULITE_CR		0x0c
#define  ULITE_CR_RST_TX	BIT(0)
#define  ULITE_CR_RST_RX	BIT(1)
#define  ULITE_CR_IE		BIT(4)

struct gxmicro_ulite_dev {
	struct uart_port uport;

	void __iomem *mmio;
};

static inline u32 gxmicro_read(struct gxmicro_ulite_dev *gdev, u32 reg)
{
	return ioread32(gdev->mmio + reg);
}

static inline void gxmicro_write(struct gxmicro_ulite_dev *gdev, u32 reg, u32 val)
{
	iowrite32(val, gdev->mmio + reg);
}

static int gxmicro_ulite_receive(struct gxmicro_ulite_dev *gdev, int stat)
{
	struct uart_port *uport = &gdev->uport;
	struct tty_port *tport = &uport->state->port;
	unsigned char ch = 0;
	char flag = TTY_NORMAL;

	if ((stat & (ULITE_SR_RXVALID | ULITE_SR_OVERRUN
			| ULITE_SR_FRAME)) == 0)
		return 0;

	/* stats */
	if (stat & ULITE_SR_RXVALID) {
		uport->icount.rx++;
		ch = gxmicro_read(gdev, ULITE_RX);

		if (stat & ULITE_SR_PARITY)
			uport->icount.parity++;
	}

	if (stat & ULITE_SR_OVERRUN)
		uport->icount.overrun++;

	if (stat & ULITE_SR_FRAME)
		uport->icount.frame++;

	/* drop byte with parity error if IGNPAR specificed */
	if (stat & uport->ignore_status_mask & ULITE_SR_PARITY)
		stat &= ~ULITE_SR_RXVALID;

	stat &= uport->read_status_mask;

	if (stat & ULITE_SR_PARITY)
		flag = TTY_PARITY;


	stat &= ~uport->ignore_status_mask;

	if (stat & ULITE_SR_RXVALID)
		tty_insert_flip_char(tport, ch, flag);

	if (stat & ULITE_SR_FRAME)
		tty_insert_flip_char(tport, 0, TTY_FRAME);

	if (stat & ULITE_SR_OVERRUN)
		tty_insert_flip_char(tport, 0, TTY_OVERRUN);

	return 1;
}

static int gxmicro_ulite_transmit(struct gxmicro_ulite_dev *gdev, int stat)
{
	struct uart_port *uport = &gdev->uport;
	struct circ_buf *xmit  = &uport->state->xmit;

	if (stat & ULITE_SR_TXFULL)
		return 0;

	if (uport->x_char) {
		gxmicro_write(gdev, ULITE_TX, uport->x_char);
		uport->x_char = 0;
		uport->icount.tx++;
		return 1;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(uport))
		return 0;

	gxmicro_write(gdev, ULITE_TX, xmit->buf[xmit->tail]);

	xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE-1);
	uport->icount.tx++;

	/* wake up */
	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(uport);

	return 1;
}

/* ****************************** Platform ****************************** */

static irqreturn_t gxmicro_ulite_irq(int irq, void *data)
{
	struct gxmicro_ulite_dev *gdev = data;
	struct uart_port *uport = &gdev->uport;
	int stat, busy, n = 0;
	unsigned long flags;

	do {
		spin_lock_irqsave(&uport->lock, flags);
		stat = gxmicro_read(gdev, ULITE_SR);
		busy  = gxmicro_ulite_receive(gdev, stat);
		busy |= gxmicro_ulite_transmit(gdev, stat);
		spin_unlock_irqrestore(&uport->lock, flags);
		n++;
	} while (busy);

	if (n < 1)
		return IRQ_NONE;

	tty_flip_buffer_push(&uport->state->port);
	return IRQ_HANDLED;
}

static int gxmicro_plat_init(struct platform_device *pdev)
{
	struct gxmicro_ulite_dev *gdev = platform_get_drvdata(pdev);
	struct resource *res;
	int irq;
	int ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gdev->mmio = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gdev->mmio))
		return PTR_ERR(gdev->mmio);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get irq\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, gxmicro_ulite_irq,
			IRQF_SHARED, KBUILD_MODNAME, gdev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request irq\n");
		return ret;
	}

	gdev->uport.iobase = 1; /* mark port in use */
	gdev->uport.membase = gdev->mmio;
	gdev->uport.irq = irq;
	gdev->uport.iotype = UPIO_MEM;
	gdev->uport.mapbase = res->start;

	return 0;
}

/* ****************************** UART ****************************** */
#define ULITE_NAME		"ttyUL"
#define ULITE_MAJOR		204
#define ULITE_MINOR		187
#define ULITE_NR_UARTS		1

static struct uart_driver gxmicro_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= KBUILD_MODNAME,
	.dev_name	= ULITE_NAME,
	.major		= ULITE_MAJOR,
	.minor		= ULITE_MINOR,
	.nr		= ULITE_NR_UARTS,
};

/* ****************************** UART ops ****************************** */

static unsigned int gxmicro_tx_empty(struct uart_port *uport)
{
	struct gxmicro_ulite_dev *gdev = uport->private_data;
	unsigned long flags;
	unsigned int ret;

	spin_lock_irqsave(&uport->lock, flags);
	ret = gxmicro_read(gdev, ULITE_SR);
	spin_unlock_irqrestore(&uport->lock, flags);

	return ret & ULITE_SR_TXEMPTY ? TIOCSER_TEMT : 0;
}

static void gxmicro_set_mctrl(struct uart_port *uport, unsigned int mctrl)
{
	/* N/A */
}

static unsigned int gxmicro_get_mctrl(struct uart_port *uport)
{
	return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static void gxmicro_stop_tx(struct uart_port *uport)
{
	/* N/A */
}

static void gxmicro_start_tx(struct uart_port *uport)
{
	struct gxmicro_ulite_dev *gdev = uport->private_data;

	gxmicro_ulite_transmit(gdev, gxmicro_read(gdev, ULITE_SR));
}

static void gxmicro_stop_rx(struct uart_port *uport)
{
	/* don't forward any more data (like !CREAD) */
	uport->ignore_status_mask = ULITE_SR_RXVALID | ULITE_SR_PARITY
		| ULITE_SR_FRAME | ULITE_SR_OVERRUN;
}

static void gxmicro_break_ctl(struct uart_port *uport, int ctl)
{
	/* N/A */
}

static int gxmicro_startup(struct uart_port *uport)
{
	struct gxmicro_ulite_dev *gdev = uport->private_data;

	gxmicro_write(gdev, ULITE_CR, ULITE_CR_RST_RX | ULITE_CR_RST_TX);
	gxmicro_write(gdev, ULITE_CR, ULITE_CR_IE);

	return 0;
}

static void gxmicro_shutdown(struct uart_port *uport)
{
	struct gxmicro_ulite_dev *gdev = uport->private_data;

	gxmicro_write(gdev, ULITE_CR, 0);
	gxmicro_read(gdev, ULITE_CR);
}

#define ULITE_FLAGS	CS8
#define ULITE_BAUD	115200

static void gxmicro_set_termios(struct uart_port *uport,
			struct ktermios *termios, struct ktermios *old)
{
	unsigned long flags;

	/* Set termios to what the hardware supports */
	termios->c_iflag &= ~BRKINT;
	termios->c_cflag &= ~(CSTOPB | PARENB | PARODD | CSIZE);
	termios->c_cflag |= ULITE_FLAGS;
	tty_termios_encode_baud_rate(termios, ULITE_BAUD, ULITE_BAUD);

	spin_lock_irqsave(&uport->lock, flags);

	uport->read_status_mask = ULITE_SR_RXVALID |
				ULITE_SR_OVERRUN |
				ULITE_SR_TXFULL;

	if (termios->c_iflag & INPCK)
		uport->read_status_mask |=
			ULITE_SR_PARITY | ULITE_SR_FRAME;

	uport->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		uport->ignore_status_mask |= ULITE_SR_PARITY
			| ULITE_SR_FRAME | ULITE_SR_OVERRUN;

	/* ignore all characters if CREAD is not set */
	if ((termios->c_cflag & CREAD) == 0)
		uport->ignore_status_mask |=
			ULITE_SR_RXVALID | ULITE_SR_PARITY
			| ULITE_SR_FRAME | ULITE_SR_OVERRUN;

	/* update timeout */
	uart_update_timeout(uport, termios->c_cflag, ULITE_BAUD);

	spin_unlock_irqrestore(&uport->lock, flags);
}

static const char *gxmicro_type(struct uart_port *uport)
{
	return uport->type == PORT_UARTLITE ? "uartlite" : NULL;
}

static void gxmicro_release_port(struct uart_port *uport)
{
	/* NA */
}

static int gxmicro_request_port(struct uart_port *uport)
{
	return 0; /* NA */
}

static void gxmicro_config_port(struct uart_port *uport, int flags)
{
	uport->type = PORT_UARTLITE;
}

static int gxmicro_verify_port(struct uart_port *uport, struct serial_struct *ser)
{
	/* we don't want the core code to modify any port params */
	return -EINVAL;
}

#ifdef CONFIG_CONSOLE_POLL
static int gxmicro_get_poll_char(struct uart_port *uport)
{
	struct gxmicro_ulite_dev *gdev = uport->private_data;

	if (!(gxmicro_read(gdev, ULITE_SR) & ULITE_SR_RXVALID))
		return NO_POLL_CHAR;

	return gxmicro_read(gdev, ULITE_RX);
}

static void gxmicro_put_poll_char(struct uart_port *uport, unsigned char ch)
{
	struct gxmicro_ulite_dev *gdev = uport->private_data;

	while (gxmicro_read(gdev, ULITE_SR) & ULITE_SR_TXFULL)
		cpu_relax();

	/* write char to device */
	gxmicro_write(gdev, ULITE_TX, ch);
}
#endif

static const struct uart_ops gxmicro_uart_ops = {
	.tx_empty	= gxmicro_tx_empty,
	.set_mctrl	= gxmicro_set_mctrl,
	.get_mctrl	= gxmicro_get_mctrl,
	.stop_tx	= gxmicro_stop_tx,
	.start_tx	= gxmicro_start_tx,
	.stop_rx	= gxmicro_stop_rx,
	.break_ctl	= gxmicro_break_ctl,
	.startup	= gxmicro_startup,
	.shutdown	= gxmicro_shutdown,
	.set_termios	= gxmicro_set_termios,
	.type		= gxmicro_type,
	.release_port	= gxmicro_release_port,
	.request_port	= gxmicro_request_port,
	.config_port	= gxmicro_config_port,
	.verify_port	= gxmicro_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char	= gxmicro_get_poll_char,
	.poll_put_char	= gxmicro_put_poll_char,
#endif
};

#define ULITE_FIFO_SZ	16
#define ULITE_REG_SHIF	2
#define ULITE_LINE	0

static int gxmicro_uart_init(struct platform_device *pdev)
{
	struct gxmicro_ulite_dev *gdev = platform_get_drvdata(pdev);
	struct uart_port *uport = &gdev->uport;
	int ret;

	uport->fifosize = ULITE_FIFO_SZ;
	uport->regshift = ULITE_REG_SHIF;
	uport->ops = &gxmicro_uart_ops;
	uport->flags = UPF_BOOT_AUTOCONF;
	uport->dev = &pdev->dev;
	uport->type = PORT_UNKNOWN;
	uport->line = ULITE_LINE;
	uport->private_data = gdev;

	ret = uart_register_driver(&gxmicro_uart_driver);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register uart driver\n");
		return ret;
	}

	ret = uart_add_one_port(&gxmicro_uart_driver, uport);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add uart port\n");
		goto err_uart_port;
	}

	return 0;

err_uart_port:
	uart_unregister_driver(&gxmicro_uart_driver);
	return ret;
}

static void gxmicro_uart_fini(struct platform_device *pdev)
{
	struct gxmicro_ulite_dev *gdev = platform_get_drvdata(pdev);

	uart_remove_one_port(&gxmicro_uart_driver, &gdev->uport);
	uart_unregister_driver(&gxmicro_uart_driver);
}

/* ****************************** Platform Init & Fini ****************************** */

static int gxmicro_ulite_probe(struct platform_device *pdev)
{
	struct gxmicro_ulite_dev *gdev;
	int ret;

	gdev = devm_kzalloc(&pdev->dev, sizeof(struct gxmicro_ulite_dev), GFP_KERNEL);
	if (!gdev)
		return -ENOMEM;

	platform_set_drvdata(pdev, gdev);

	ret = gxmicro_plat_init(pdev);
	if (ret)
		return ret;

	ret = gxmicro_uart_init(pdev);
	if (ret)
		return ret;

	return ret;
}

static int gxmicro_ulite_remove(struct platform_device *pdev)
{
	gxmicro_uart_fini(pdev);

	return 0;
}

static const struct platform_device_id gxmicro_ulite_ids[] = {
	{ .name = KBUILD_MODNAME },
	{ /* END OF LIST */ }
};
MODULE_DEVICE_TABLE(platform, gxmicro_ulite_ids);

static struct platform_driver gxmicro_ulite_driver = {
	.probe = gxmicro_ulite_probe,
	.remove = gxmicro_ulite_remove,
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.id_table = gxmicro_ulite_ids,
};
module_platform_driver(gxmicro_ulite_driver);

MODULE_DESCRIPTION("GXMicro UARTLite Controller driver");
MODULE_AUTHOR("DongXiong Zheng <zhengdongxiong@gxmicro.cn>");
MODULE_VERSION("v1.0");
MODULE_LICENSE("GPL");
