PWD := $(shell pwd)
obj-m += mylinuxdrone.o
obj-m += mld_pru_cntrl_channel.o
obj-m += mld_pru_imu_channel.o
obj-m += mld_pru_rc_channel.o

CROSS_ := /hd/linuxlab/bb-kernel/dl/gcc-linaro-6.4.1-2017.11-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
KERNEL_ := /hd/linuxlab/beagleboard/linux
SUBDIRS_ :=/hd/eworkspace/mylinuxdrone

all:
	make ARCH=arm CROSS_COMPILE=$(CROSS_) -C $(KERNEL_) SUBDIRS=$(SUBDIRS_) modules
clean:
	make -C $(KERNEL_) SUBDIRS=$(SUBDIRS_) clean

