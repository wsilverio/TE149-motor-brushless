//--------------------------------------------------------------------------
// bibliotecas
#include <msp430.h>
#include "serial_uart.h"

//==========================================================================
// SERIAL CONFIG
// funcao: configura a comunicacao serial UART
// retorno: nenhum
// parametros: nenhum
// constantes:
//      SERIAL_DBG: se definida, habilita msgs de debug
//      SERIAL_SMCLK: sub-main clock (Hz)
//      SERIAL_BAUD: taxa de comunicacao (bps)
//      SERIALRXPIN: pino de recepcao
//      SERIALTXPIN: pino de transmissao
//==========================================================================
void serial_config(){
    // software reset enabled
    UCA0CTL1 |= UCSWRST;
    // clock source select: SMCLK
    UCA0CTL1 |= UCSSEL_2;
    uint16_t ucbr;
    uint8_t ucbrs;
    uint8_t ucbrf;

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
#elif SERIAL_BAUD == 230400
    ucbr = 4;
    ucbrs = 5;
    ucbrf = 3;
    UCA0MCTL |= (ucbrf<<4) | UCOS16;
#elif SERIAL_BAUD == 460800
    ucbr = 2;
    ucbrs = 3;
    ucbrf = 2;
    UCA0MCTL |= (ucbrf<<4) | UCOS16;
#elif SERIAL_BAUD == 256000
    ucbr = 62;
    ucbrs = 4;
#else
#error Baudrate not implemented
#endif
#else
#error SMCLK not implemented
#endif

    UCA0BR0 = (ucbr&0x00FF);
    UCA0BR1 = (ucbr&0xFF00)>>8;
    UCA0MCTL |= (ucbrs<<1);    

    // function select registers
    P1SEL  |= (SERIALRXPIN|SERIALTXPIN);
    P1SEL2 |= (SERIALRXPIN|SERIALTXPIN);
    // software reset disabled
    UCA0CTL1 &= ~UCSWRST;
    // receive interrupt enabled
    IE2 |= UCA0RXIE;
}

//==========================================================================
// SERIAL PRINT BYTE
// funcao: envia um byte pela porta serial UART
// retorno: nenhum
// parametros: caractere ASCII (const int8_t)
// constantes: nenhuma
//==========================================================================
void serial_print_byte(const char data){
    while(!(IFG2 & UCA0TXIFG));
    UCA0TXBUF = data;
}

//==========================================================================
// SERIAL PRINT STRING
// funcao: envia uma c_string pela porta serial UART
// retorno: nenhum
// parametros: string ASCII (const char*)
// constantes: nenhuma
//==========================================================================
void serial_print_string(const char* data){
    while(*data){
        serial_print_byte(*data);
        data++;
    }
}
