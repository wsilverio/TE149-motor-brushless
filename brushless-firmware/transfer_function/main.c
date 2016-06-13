//--------------------------------------------------------------------------
#include <msp430.h>
#include <stdlib.h> // stdlib
#include <stdint.h> // uint8_t
#include <stdbool.h> // bool
#include "serial_uart.h"
//--------------------------------------------------------------------------
// GPIO
#define REDLEDPIN BIT0 // P1.0 (RED LED)
#define BUTTONPIN BIT3 // P1.3 (S2)
#define MOTORINPIN BIT4 // P1.4
#define MOTOROUTPIN BIT6 // P1.6 / TA01 / GREEN LED
//servo
#define SERVOSTOPPULSE 1000 // 1ms
#define SERVOMAXPULSE 2000 // 2ms
// #define RPMMAX 6000
// #define RPMMIN 2000
// amostragem
#define SAMPLINGINTERVAL (10000<<1)-1 // 10 ms
// media exponencial movel
#define NM 20.0 // numero de medias
#define ALPHA NM/(NM+1) // coeficiente exponencial
//--------------------------------------------------------------------------
// clock
void clock_config();
// servo
void servo_config();
inline void servo_write_pulse(uint16_t ms);
// cronometro
void cronometro_config();
// GPIO
void gpio_config();
// amostragem
void sampling_config();
// misc
void itoa_base_10(uint32_t num, char* str);
void delay_ms(uint16_t ms);
//--------------------------------------------------------------------------
// servo
volatile uint16_t nextPulse = 0;
// amostragem
volatile bool sendSerial = false;
// timer
volatile uint16_t timerCount = 0;
volatile uint16_t timerOverflow = 0;
volatile uint16_t overTimer = 0;
// calculo
char strSerialValue[8] = {'\0'}; // string de uso geral
// GPIO
volatile bool buttonFlag = false;
//--------------------------------------------------------------------------
int main(){
    // disable watchdog timer
    WDTCTL = WDTPW | WDTHOLD;

    // configs iniciais
    clock_config();
    serial_config();
    servo_config();
    sampling_config();
    cronometro_config();
    gpio_config();

    uint16_t rpm[2] = {0}; // velocidade em RPM
    char strRpmValue[16]; // string de uso geral
    uint16_t rpmInst = 0;

    __enable_interrupt();

    serial_print_string("\n*** START ***\n");

    while(1){
        if(sendSerial){
            sendSerial = false;
            // calcula a velocidade
//            float delta_t = (62.5e-9*timerCount + 3.125e-3*overTimer);
//            if(delta_t != 0){
//                rpmInst = (uint16_t)((60.0f/7.0f)/delta_t); // 60s/polos
//            }
            // media exponencial movel
//            rpm[1] = ALPHA*rpm[0]+(1-ALPHA)*rpmInst;
//            rpm[0] = rpm[1];
            // converte para string
//            itoa_base_10(rpm[1], strRpmValue);
            // envia pela serial
//            serial_print_string(strRpmValue);
//            serial_print_byte('\n');
            itoa_base_10(timerCount, strRpmValue);
            serial_print_string(strRpmValue);
            serial_print_byte(',');
            itoa_base_10(overTimer, strRpmValue);
            serial_print_string(strRpmValue);
            serial_print_byte('\n');
        }
    }

    return 0;
}
//--------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR(void)
#else
#error Compiler not supported!
#endif
{
    uint8_t j; // for loops
    static uint8_t i = 0; // char index
    static bool overflow = false;

    char val = UCA0RXBUF;

    if (7 == i && '\n' != val){
        i = 0;
        overflow = true;
        for (j=7; j; j--){
            strSerialValue[j]=0;
        }
    }else if(7 >= i){
        strSerialValue[i++] = val;
        if ('\n' == val){
            if(!overflow){
                strSerialValue[i-1] = '\0';
                uint16_t pulseWidth = atoi(strSerialValue);
                if(SERVOSTOPPULSE <= pulseWidth && SERVOMAXPULSE >= pulseWidth ){
                    serial_print_byte('*');
                    servo_write_pulse(pulseWidth);
                }
            }else{
                overflow=false;
            }
            i = 0;
            for (j=7; j; j--){
                strSerialValue[j]=0;
            }
        }
    }
}
//--------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void PORT1_VECTOR_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT1_VECTOR))) PORT1_VECTOR_ISR(void)
#else
#error Compiler not supported!
#endif
{
    if(P1IFG & BUTTONPIN){
        buttonFlag = true;
        // limpa flag de interrupcao
        P1IFG &= ~BUTTONPIN;
    }else if(P1IFG & MOTORINPIN){
        // captura tempo atual
        if(!sendSerial){
            overTimer = timerOverflow;
            timerCount = TA1R;
        }

        // zera os contadores
        timerOverflow = TA1R = 0;

        // limpa flag de interrupcao
        P1IFG &= ~MOTORINPIN;
    }
}

//--------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer_A0(void)
#else
#error Compiler not supported!
#endif
{
//    TA0CCR1 = nextPulse;
    TA0CCTL0 &= ~CCIFG;
}
//--------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
__interrupt void Timer1_A0(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER1_A0_VECTOR))) Timer1_A0(void)
#else
#error Compiler not supported!
#endif
{
    // incrementa a flag estouro do timer
    ++timerOverflow;
    TA1CCTL0 &= ~CCIFG;
}
//--------------------------------------------------------------------------
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer0_A1 (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(TA0IV, TA0IV_TAIFG)){
//        case TA0IV_NONE: break;
//        case TA0IV_TACCR1: break;
        case TA0IV_TACCR2:
            sendSerial = true; // habilita envio
            TA0CCR2 += SAMPLINGINTERVAL; // prox. envio
            break;
//        case TA0IV_6: break;
//        case TA0IV_8: break;
        case TA0IV_TAIFG:
            sendSerial = true; // habilita envio
            TA0CCR1 = nextPulse; // atualiza pwm
            TA0CCR2 = SAMPLINGINTERVAL; // prox.envio
            break;
    }
}
//--------------------------------------------------------------------------
void clock_config(){
    if (0xFF == CALBC1_16MHZ){
        while(1);
    }

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
}
//--------------------------------------------------------------------------
void servo_config(){
    TA0CTL |= TASSEL_2 | ID_3 | MC_1 | TAIE; // SMCLK, DIV(8), UP CCR0, INT
//    TA0CCTL0 |= CCIE; //INT
    TA0CCR0 = 40000-1; // periodo (16Mhz/8/1000ms) * 20ms = 40_000 ciclos
    TA0CCTL1 |= OUTMOD_7; // reset/set
    TA0CCR1 = 0; // reset
}
//--------------------------------------------------------------------------
inline void servo_write_pulse(uint16_t ms){
    nextPulse = ms<<1; // prox. pwm
}
//--------------------------------------------------------------------------
void gpio_config(){
     P1DIR |= (MOTOROUTPIN|REDLEDPIN); // saidas
     P1SEL |= (MOTOROUTPIN); // pwm out
     P1REN |= (MOTORINPIN|BUTTONPIN); // habilita resistores
     P1OUT |= (MOTORINPIN|BUTTONPIN); // resistores pullup
     P1OUT &= ~(REDLEDPIN); // limpa as saidas

     P1IE |= (MOTORINPIN|BUTTONPIN); // habilita interrupcao
     P1IES |= (MOTORINPIN|BUTTONPIN); // borda de descida
     P1IFG &= ~(MOTORINPIN|BUTTONPIN); // limpa IFG
}
//--------------------------------------------------------------------------
void cronometro_config(){
    // configura o timer A1
    TA1CTL = TASSEL_2 | ID_0 | MC_1; // smclk, div 1, up CCR0
    TA1CCTL0 |= CCIE; // interrupcao por comparacao
    TA1CCR0 = 50000-1; // @16MHz: 1/320 = 3,125ms para estourar
}
//--------------------------------------------------------------------------
void sampling_config(){
    TA0CCTL2 |= CCIE; //INT
    TA0CCR2 = SAMPLINGINTERVAL; // tempo de amostragem
}
//--------------------------------------------------------------------------
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
//--------------------------------------------------------------------------
void delay_ms(uint16_t ms){
    while (ms--) {
        __delay_cycles(16000);
    }
}
