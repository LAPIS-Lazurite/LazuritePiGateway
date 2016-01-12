KERNEL_SRC=/lib/modules/$(shell uname -r)/build
CFILES = drv-802154e.c mac-802154e.c phy-bp3596.c i2c-bp3596.c spi-bp3596.c ieee802154e.c
# CFILES = drv-802154e.c mac-802154e.c phy-bp3596.c i2c-bp3596.c spi-bp3596.c 

obj-m += DRV_802154.o
DRV_802154-objs := $(CFILES:.c=.o)

all:
	make -C $(KERNEL_SRC) SUBDIRS=$(PWD) modules

clean:
	make -C $(KERNEL_SRC) SUBDIRS=$(PWD) clean

clean-files := *.o *.ko *.mod.[co] *~
