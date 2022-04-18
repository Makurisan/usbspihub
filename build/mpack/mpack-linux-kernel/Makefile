#
# Copyright (c) 2021 Nicholas Fraser
#
# This program is dual licensed under the terms of the MIT license OR the GNU
# General Public License version 2, your choice.
#
# This program is distributed WITHOUT ANY WARRANTY; without even the implied
# warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# attached licenses for details.
# mpack/test/unit/src/{*.o,.*.o.cmd}

KDIR ?= ~/rpi/linux-5.10.17-v7

all:
	make -C $(KDIR) \
	ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
			M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean \
		ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
	rm -f mpack/src/mpack/{*.o,.*.o.cmd} 


#obj-m  := v-hub.o 
#v-hub-objs := vusb_core.o vusb_udc.o vusb_spi.o  vusb_ep.o vusb_helper.o vusb_chrdev.o
#
#KERNEL_DIR ?= ~/rpi/linux-5.10.17-v7
#
#all:
#	        make -C $(KERNEL_DIR) \
#	                ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
#					                M=$(PWD) modules
#clean:
#	        make -C $(KERNEL_DIR) \
#			ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- \
#						      M=$(PWD) clean
#