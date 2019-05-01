KERNEL_SOURCE=~/bbb-b2qt-yocto/build/build-beaglebone/tmp/work-shared/beaglebone/kernel-source

obj-m += ti-tlv2553.o

all:
	make -C ${KERNEL_SOURCE} M=$(PWD) modules

clean:
	make -C ${KERNEL_SOURCE} M=$(PWD) clean
