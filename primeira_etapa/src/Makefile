all:	firmware

firmware:	main.c
	msp430-gcc --std=c99 -Os -mmcu=msp430g2553 main.c -o firmware.elf

flash:  firmware
	mspdebug rf2500 "prog firmware.elf"

install_dependencies:
	apt-get install mspdebug gcc-msp430

clean:
	if [ -e firmware.elf ]; then rm firmware.elf; fi