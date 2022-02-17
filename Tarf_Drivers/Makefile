TARGET := Tarf_ar0330_stereo_camera
#TARGET := hello
TARGET_IP := 192.168.1.116
#KERNEL := /home/e.bazaras/Projects/viper/Linux_for_Tegra/source/public/kernel/kernel-4.9
KERNEL := /home/e.bazaras/Downloads/Linux_for_Tegra/source/public/kernel/kernel-4.9
PWD := $(shell pwd)
obj-m += $(TARGET).o
CROSS := /home/e.bazaras/Projects/viper/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-
# aarch64-linux-gnu-
ccflags-y := -Wno-incompatible-pointer-types -Wno-int-conversion -Wno-overflow -O2 -DMODULE -D__KERNEL__ -D__NEED_MEDIA_LEGACY_API -DCONFIG_MEDIA_CONTROLLER_DIS \
	-I$(KERNEL)/../nvidia/drivers/media/platform/tegra

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

rmmod:
	-ssh root@$(TARGET_IP) rmmod $(TARGET).ko

test:
	ssh root@$(TARGET_IP) cat /dev/video0

reboot:
	ssh root@$(TARGET_IP) reboot

asm: build
	/home/e.bazaras/Projects/viper/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-objdump -D Tarf_ar0330_stereo_camera.ko > Tarf_ar0330_stereo_camera.asm
