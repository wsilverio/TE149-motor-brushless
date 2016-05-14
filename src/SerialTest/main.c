#include <msp430.h>     // PxOUT, PxIN, ...
#include <stdint.h>     // int8_t, int16_t, ...
#include <stdbool.h>    // bool
#include <stdlib.h>     // itoa

void servo_config();
void serial_config();
void serial_print_byte(const int8_t data);
void serial_print_string(const char* data);
void servo_write_pulse(const uint16_t ms);

#define SERIALRXPIN BIT1
#define SERIALTXPIN BIT2
#define MOTOROUTPIN BIT6

char strSerialValue[8];


void main(){
    // desabilita watchdog
    WDTCTL = WDTPW + WDTHOLD;

    //16Mhz
    if (0xFF == CALBC1_16MHZ){
        while(1);
    }

    DCOCTL = 0; //calibra 16MHz
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;

    P1DIR |= (MOTOROUTPIN);    // saidas
    P1SEL |= (MOTOROUTPIN);    // pwm out

    // config. perifericos
    serial_config();
    servo_config();

    __enable_interrupt();

    while(1){}
}

void servo_config(){
    TA0CTL = TASSEL_2 | ID_3 | MC_1;    // SMCLK, DIV(8), UP CCR0
    TA0CCTL1 |= OUTMOD_7;               // PWM set/reset
    TA0CCR0 = 20000-1;                  // periodo: (16Mhz / 8 / 1000ms) * 20ms
    TA0CCR1 = 0;                        // reset
}

void serial_config(){
    P1SEL  |= (SERIALRXPIN | SERIALTXPIN);
    P1SEL2 |= (SERIALRXPIN | SERIALTXPIN);

    UCA0CTL1 |= UCSWRST;
    UCA0CTL0 &= ~(UCMODE1 + UCMODE0 + UCSYNC);
    UCA0CTL1 |= UCSSEL_2;
    UCA0MCTL &= ~UCOS16;

    // 9600 bps
    UCA0BR1 = 0x06;
    UCA0BR0 = 0x82;
    UCA0MCTL |= 0x0A;

    UCA0CTL1 &= ~UCSWRST;

    IE2 |= UCA0RXIE;
}

void serial_print_byte(const int8_t data){
    // aguarda buffer vazio
    while(!(IFG2 & UCA0TXIFG));
    // escreve o dado no registrador
    UCA0TXBUF = data;
}

void serial_print_string(const char* data){
    while(*data){
        serial_print_byte(*data);
        data++;
    }
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void serial_receive(){

    static uint8_t i = 0;

    static bool overflow = false;

    char val = UCA0RXBUF;


    if (i==7 && val != '\n'){

        overflow = true;
        serial_print_string("Overflow\n");
        
        for (uint8_t j=0; j<8; j++){
            strSerialValue[j]=0;
        }
        i=0;
    }else if(i<=7){
        
        strSerialValue[i] = val;
        i++;

        if (val=='\n'){
            if(!overflow){
                strSerialValue[i-1]=0;
                int valint = atoi(strSerialValue);
                serial_print_byte('*');
                serial_print_string(strSerialValue);
                serial_print_byte('*');
            }else{overflow=false;}
        
            i=0;
            for (uint8_t j=0; j<8; j++){
                strSerialValue[j]=0;
            }
        }
    }
}

void servo_write_pulse(const uint16_t ms){
    TA0CCR1 = 2*ms;
}