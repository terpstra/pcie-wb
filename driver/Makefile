#export PATH=$PATH:/share/eldk/bin:/share/eldk/usr/bin
#export CROSS_COMPILE=ppc_4xxFP-
#export ARCH=powerpc

# This is useful if cross-compiling. Taken from kernel Makefile (CC changed)
#AS      =$(CROSS_COMPILE)as
#LD      =$(CROSS_COMPILE)ld
#CC      =$(CROSS_COMPILE)gcc
#CPP     =$(CC) -E
#AR      =$(CROSS_COMPILE)ar
#NM      =$(CROSS_COMPILE)nm
#STRIP   =$(CROSS_COMPILE)strip
#OBJCOPY =$(CROSS_COMPILE)objcopy
#OBJDUMP =$(CROSS_COMPILE)objdump



KERNELDIR ?= /usr/src/linux-3.2.14 #/common/usr/embedded/kernel/scu/linux-2.6.33.6/
ifneq ($(KERNELRELEASE),)
CFLAGS_pcie_wb.o = -msse2
	obj-m	:= pcie_wb.o wishbone.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build
	PWD       := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

endif

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions

