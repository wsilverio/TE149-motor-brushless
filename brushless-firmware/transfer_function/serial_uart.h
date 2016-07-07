#ifndef _SERIAL_UART_H_
#define _SERIAL_UART_H_

//
// * TODO:
// *    serial_config(): implementacao para outrs clk e bps
// *    serial_read_bytes()
// *

#include <stdint.h>
#include <stdbool.h>

#define SERIAL_SMCLK 16000000
#define SERIAL_BAUD 115200

void serial_config();
void serial_print_byte(const int8_t data);
void serial_print_string(const char* data);
// bool serial_available();
// char serial_read();

#endif
