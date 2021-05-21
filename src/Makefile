obj-m+= spi.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules 
	
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean

install:
	sudo insmod ./spi.ko
uninstall:
	sudo rmmod ./spik.ko

run:
	./test_driver.exe
