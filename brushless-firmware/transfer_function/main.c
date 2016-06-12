#include <msp430.h> 
#include "serial_uart.h"

#define REDLEDPIN       BIT0    // P1.0 (RED LED)
#define BUTTONPIN       BIT3    // P1.3 (S2)
#define SERIALRXPIN     BIT1    // P1.1
#define SERIALTXPIN     BIT2    // P1.2
#define MOTORINPIN      BIT4    // P1.4
#define MOTOROUTPIN     BIT6    // P1.6 / TA01 / GREEN LED

#define SAMPLINGINTERVAL (4000<<1)-1 // 4 ms

void servo_config();
void sampling_config();
void itoa_base_10(uint32_t num, char* str);
void delay_ms(uint16_t ms);

int main(){
    WDTCTL = WDTPW | WDTHOLD;

    if (0xFF == CALBC1_16MHZ){
        while(1);
    }

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;

    serial_config(16000000, 9600);
    servo_config();
    sampling_config();

    // config. I/O
    P1DIR |= (MOTOROUTPIN | REDLEDPIN);    // saidas
    P1SEL |= (MOTOROUTPIN);             // pwm out
    P1REN |= MOTORINPIN;  // habilita resistores
    P1OUT |= MOTORINPIN;  // resistores pullup
    P1OUT &= ~(REDLEDPIN);   // limpa as saidas

    __enable_interrupt();

    serial_print_string("\n*** START ***\n");

    uint16_t count = 0;
    char miscStr[16]; // uso geral
    while(1){
        itoa_base_10(count++, miscStr);
        serial_print_string(miscStr);
        serial_print_byte('\n');
        delay_ms(500);
    }

    return 0;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR (void)
#else
#error Compiler not supported!
#endif
{
    char data = UCA0RXBUF;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A0 (void)
#else
#error Compiler not supported!
#endif
{
    TA1CCTL0 &= ~CCIFG;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer_A1 (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(TA0IV, TA0IV_TAIFG)){
//        case TA0IV_NONE: break;
//        case TA0IV_TACCR1: break;
        case TA0IV_TACCR2:
            TA0CCR2 += SAMPLINGINTERVAL;
            break;
//        case TA0IV_6: break;
//        case TA0IV_8: break;
        case TA0IV_TAIFG:
            TA0CCR2 = SAMPLINGINTERVAL;
            break;
    }
}

void servo_config(){
    TA0CTL |= TASSEL_2 | ID_3 | MC_1 | TAIE; // SMCLK, DIV(8), UP CCR0, INT
    TA0CCTL1 |= OUTMOD_7; // reset/set
    TA0CCR0 = 40000-1; // periodo: (16Mhz / 8 / 1000ms) * 20ms = 40_000 ciclos
    TA0CCR1 = 0; // reset
}

void sampling_config(){
    TA0CCTL2 |= CCIE; //INT
    TA0CCR2 = SAMPLINGINTERVAL;
}

void itoa_base_10(uint32_t num, char* str){

    char* ptr = str, *ptr1 = str, char_tmp;
    int int_tmp;

    do{
        int_tmp = num;
        num /= 10;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (int_tmp - num * 10)];
    }while(num);

    if(int_tmp < 0) *ptr++ = '-';

    *ptr-- = '\0';

    while(ptr1 < ptr){
        char_tmp = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = char_tmp;
    }
}

void delay_ms(uint16_t ms){
    while (ms--) {
        __delay_cycles(16000);
    }
}
