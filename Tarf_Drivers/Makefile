TARGET := Tarf_ar0330_stereo_camera
TARGET_IP := 192.168.1.116
#KERNEL_ROOT := /home/e.bazaras/Projects/viper/R28.2.1/Linux_for_Tegra/source/public/kernel
KERNEL_ROOT := /home/e.bazaras/Downloads/Linux_for_Tegra/source/public/kernel
#KERNEL := $(KERNEL_ROOT)/kernel-4.4
KERNEL := $(KERNEL_ROOT)/kernel-4.9
PWD := $(shell pwd)
obj-m += $(TARGET).o
#CROSS := aarch64-linux-gnu-
CROSS := /home/e.bazaras/Projects/viper/gcc-linaro-5.3-2016.02-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
ccflags-y := -Wno-incompatible-pointer-types -Wno-int-conversion -Wno-overflow -O2 -DMODULE -D__KERNEL__ -D__NEED_MEDIA_LEGACY_API \
	-I$(KERNEL_ROOT)/nvidia/drivers/media/platform/tegra -I$(KERNEL)/drivers/media/platform/tegra

build:
	make ARCH=arm64 CROSS_COMPILE=$(CROSS) -C $(KERNEL) SUBDIRS=$(PWD) modules
clean:
	make -C $(KERNEL) SUBDIRS=$(PWD) clean

install:
	sudo cp $(TARGET).ko ../rootfs/root/

upload:
	scp $(TARGET).ko root@$(TARGET_IP):

insmod: upload
	ssh root@$(TARGET_IP) insmod $(TARGET).ko
	ssh root@$(TARGET_IP) modprobe nvhost-vi4

rmmod:
	-ssh root@$(TARGET_IP) rmmod $(TARGET).ko

test:
	ssh root@$(TARGET_IP) cat /dev/video0

reboot:
	ssh root@$(TARGET_IP) reboot

asm: build
	$(CROSS)objdump -Dz Tarf_ar0330_stereo_camera.ko > Tarf_ar0330_stereo_camera.asm

