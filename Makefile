all:	firmware panel

firmware:
	@echo "brushless-firmware/firmware.elf"
	@msp430-gcc --std=c99 -Os -mmcu=msp430g2553 brushless-firmware/main.c brushless-firmware/serial_uart.* -o brushless-firmware/firmware.elf

flash:	firmware
	mspdebug rf2500 "prog brushless-firmware/firmware.elf"

panel:
	@echo "brushless_panel.run"
	@c++ `sdl2-config --cflags` -I brushless-panel/third-party/imgui brushless-panel/main.cpp brushless-panel/imgui_impl_sdl.cpp brushless-panel/third-party/imgui/imgui*.cpp `sdl2-config --libs` -lGL -o brushless-panel/brushless_panel.run

install_dependencies:
	apt-get install mspdebug gcc-msp430

clean:
	@if [ -e brushless-firmware/firmware.elf ]; then echo "brushless-firmware/firmware.elf" && rm brushless-firmware/firmware.elf; fi
	@if [ -e brushless_panel.run ]; then echo "brushless_panel.run" && rm brushless_panel.run; fi
	@if [ -e imgui.ini ]; then echo "imgui.ini" && rm imgui.ini; fi
