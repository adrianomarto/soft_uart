obj-m += soft_uart.o

soft_uart-objs := module.o raspberry_soft_uart.o raspberry_gpio.o queue.o

LINUX = /usr/src/linux

all:
	$(MAKE) -C $(LINUX) M=$(PWD) modules

clean:
	$(MAKE) -C $(LINUX) M=$(PWD) clean

install:
	install -m 644 -c soft_uart.ko /lib/modules/`uname -r`
