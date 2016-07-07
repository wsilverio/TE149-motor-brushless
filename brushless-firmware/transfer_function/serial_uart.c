#include <msp430.h>
#include "serial_uart.h"

// #define SERIAL_DBG

#define SERIALRXPIN BIT1 // P1.1
#define SERIALTXPIN BIT2 // P1.2

// bool serialAvailable = false;
// char serialLastChar = 0;

void serial_config(){
    // software reset enabled
    UCA0CTL1 |= UCSWRST;

    // clock source select: SMCLK
    UCA0CTL1 |= UCSSEL_2;

    uint16_t ucbr;
    uint8_t ucbrs;

#ifndef SERIAL_SMCLK
#error SERIAL_SMCLK is not defined
#endif

#ifndef SERIAL_BAUD
#error SERIAL_BAUD is not defined
#endif

#if SERIAL_SMCLK == 1000000 // 1MHz
#if SERIAL_BAUD == 9600
    ucbr = 104;
    ucbrs = 1;
#elif SERIAL_BAUD == 115200
    ucbr = 8;
    ucbrs = 6;
#else
#error Baudrate not implemented
#endif
#elif SERIAL_SMCLK == 8000000 // 8MHz
#if SERIAL_BAUD == 9600
    ucbr = 833;
    ucbrs = 2;
#elif SERIAL_BAUD == 115200
    ucbr = 69;
    ucbrs = 4;
#else
#error Baudrate not implemented
#endif
#elif SERIAL_SMCLK == 16000000 // 16MHz
#if SERIAL_BAUD == 9600
    ucbr = 1666;
    ucbrs = 6;
#elif SERIAL_BAUD == 115200
    ucbr = 138;
    ucbrs = 7;
#else
#error Baudrate not implemented
#endif
#else
#error SMCLK not implemented
#endif

    UCA0BR0 = (ucbr&0x00FF);
    UCA0BR1 = (ucbr&0xFF00)>>8;
    UCA0MCTL = (ucbrs<<1);

    // function select registers
    P1SEL  |= (SERIALRXPIN|SERIALTXPIN);
    P1SEL2 |= (SERIALRXPIN|SERIALTXPIN);
    
    // software reset disabled
    UCA0CTL1 &= ~UCSWRST;
    // receive interrupt enabled
    IE2 |= UCA0RXIE;
}

void serial_print_byte(const int8_t data){
    while(!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = data;
}

void serial_print_string(const char* data){
    while(*data){
        serial_print_byte(*data);
        data++;
    }
}

// bool serial_available(){
//     return serialAvailable;
// }

// char serial_read(){
//     serialAvailable = false;
//     return serialLastChar;
// }

// #pragma vector = USCIAB0RX_VECTOR
// __interrupt void serial_receive(){
//     serialLastChar = UCA0RXBUF;
//     serialAvailable = true;
// }
