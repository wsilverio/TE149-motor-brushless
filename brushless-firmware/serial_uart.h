#ifndef _SERIAL_UART_H_
#define _SERIAL_UART_H_

#include <stdint.h>
#include <stdbool.h>

// #define SERIAL_DBG
#define SERIAL_SMCLK 16000000
#define SERIAL_BAUD 256000

#define SERIALRXPIN BIT1 // P1.1
#define SERIALTXPIN BIT2 // P1.2

void serial_config();
void serial_print_byte(const char data);
void serial_print_string(const char* data);

#endif
