obj-m += vivax-panel.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(shell pwd) clean

dkms:
	mkdir /usr/src/vivax-panel-5.12.0
	cp -rf * /usr/src/vivax-panel-5.12.0
	dkms add vivax-panel/5.12.0
	dkms install vivax-panel/5.12.0

dkms_clean:
	dkms remove vivax-panel/5.12.0 --all
	rm -rf /usr/src/vivax-panel-5.12.0

