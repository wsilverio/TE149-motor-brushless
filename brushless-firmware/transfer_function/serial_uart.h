#ifndef _SERIAL_UART_H_
#define _SERIAL_UART_H_

//
// * TODO:
// *    serial_config(): implementacao para outrs clk e bps
// *    serial_read_bytes()
// *

#include <stdint.h>
#include <stdbool.h>

void serial_config(const uint32_t smclk, const uint32_t bps);
void serial_print_byte(const int8_t data);
void serial_print_string(const char* data);
// bool serial_available();
// char serial_read();

#endif
