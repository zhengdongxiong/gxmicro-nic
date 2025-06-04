# SPDX-License-Identifier: GPL-2.0

ifneq ($(KERNELRELEASE),)

# object files to build
obj-y += mfd/
obj-y += ata/
obj-y += dma/
obj-y += spi/
obj-y += i2c/
obj-y += serial/

else

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

default: modules

help modules clean:
	$(MAKE) -C $(KDIR) M=$(PWD) $@

install: modules_install
	@:

endif
