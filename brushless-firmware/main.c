//==========================================================================
//
// TE149-motor-brushless
// Controlador PID Digital para motor brushless
//
// Disciplina TE149 - Instrumentacao Eletronica - UFPR 2016/1
// Prof. Marlio Bonfim
//
// Autores:
//    Guilherme Restani: https://github.com/GuilhermeRestani
//    Wendeurick Silverio: https://github.com/wsilverio
//   _   _ _____ ____  ____    ____   ___  _  __
//  | | | |  ___|  _ \|  _ \  |___ \ / _ \/ |/ /_
//  | | | | |_  | |_) | |_) |   __) | | | | | '_ \
//  | |_| |  _| |  __/|  _ <   / __/| |_| | | (_) |
//   \___/|_|   |_|   |_| \_\ |_____|\___/|_|\___/
//
// compilado com "msp430-gcc 4.6.3 e TI 4.4.5", C99 para mcu=msp430g2553
//
//
//
// Planta
// G =
//               7.167
//      ------------------------- * exp(-0.0684*s)
//      0.03059 s^2 + 0.334 s + 1
//
//
// Controlador
// C =
//      0.02151 s^2 + 0.2537 s + 0.7479
//      -------------------------------
//                     s
//
//
//==========================================================================

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
#define MOTOROUTPIN BIT6 // P1.6 / TA01 (GREEN LED)
// servo
#define SERVOSTOPPULSE 1000 // 1ms
#define SERVOMAXPULSE 2000  // 2ms
// #define SERVOINIPULSE 1215 // aprox 5000 rpm
// intervalo de velocidade
#define RPMMAX 8000
#define RPMMIN 2000
// amostragem
#define SAMPLINGINTERVAL (10000<<1)-1 // 10 ms
// media exponencial movel
#define NM 20.0 // numero de medias
#define ALPHA NM/(NM+1) // coeficiente exponencial
// ganhos do controlador

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
// miscelanea
void itoa_base_10(int32_t num, char* str);
void delay_ms(uint16_t ms);

//--------------------------------------------------------------------------
// servo
volatile uint16_t nextPulse = 0; // prox. pulso
// amostragem
volatile bool amostrar = false;
// timer
volatile uint16_t timerCount = 0;
volatile uint16_t timerOverflow = 0;
volatile uint16_t overTimer = 0;
// calculo
char strSerialValue[8] = {'\0'}; // string de uso geral
// GPIO
volatile bool buttonFlag = true;
// serial 
uint16_t serialVal = 0;

//==========================================================================
//
//==========================================================================
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

    serial_print_string("\n--- START ---\n");

    __enable_interrupt(); // habilita interrupcoes

    char generalStr[16] = {'\0'}; // string de uso geral
    uint16_t rpmInst = 0; // velocidade instantanea
    uint16_t rpm[2] = {0}; // velocidade em RPM (media exp movel)
    // int16_t intError = 0; // integral do erro
    // uint16_t lastSetRPM = 0;

    // loop principal
    while(1){
        // intervalo de amostragem controlado por TIMER0_A1
        if(amostrar){

            amostrar = false;

            // calcula a velocidade
            float delta_t = (62.5e-9f*timerCount + 3.125e-3f*overTimer);
            if(delta_t > 0){ // previne divisao por zero
                rpmInst = (uint16_t)(8.5714f/delta_t); // 8.5714 = 60s/7(polos motor)
            }

            // media exponencial movel
            rpm[1] = ALPHA*rpm[0]+(1-ALPHA)*rpmInst;
            rpm[0] = rpm[1];

            itoa_base_10(serialVal, generalStr);
            serial_print_string(generalStr);
            serial_print_byte('\t');           

            itoa_base_10(rpm[0], generalStr);
            serial_print_string(generalStr);
            serial_print_byte('\n');

        }
    }

    return 0;
}

//==========================================================================
// USCI0RX ISR
// funcao: servico de interrupcao UART. Recebe valores pela serial e concatena
//         na string strSerialValue. Limite de digitos 6 (+ end line)
//         Se receber um valor valido, atualiza o set-point
// retorno: nenhum
// parametros: nenhum
// constantes:
//      SERIAL_DBG: debug
//      MOTORINPIN: saida do filtro
//==========================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCIAB0RX_VECTOR))) USCI0RX_ISR(void)
#else
#error Compiler not supported!
#endif
{
    static uint8_t i = 0;

    static bool overflow = false;

    char val = UCA0RXBUF;

    if (7 == i && '\n' != val){

        overflow = true;

#ifdef SERIAL_DBG
        serial_print_string("\n*** buffer overflow ***\n");
#endif

        for (uint8_t j=7; j; j--){
            strSerialValue[j]=0;
        }

        i = 0;
    }else if(7 >= i){

        strSerialValue[i] = val;
        i++;

        if ('\n' == val){
            if(!overflow){
                strSerialValue[i-1] = '\0';
                serialVal = atoi(strSerialValue);
                
                if(SERVOMAXPULSE < serialVal){
                    serialVal = SERVOMAXPULSE;
                }else if(SERVOSTOPPULSE > serialVal){
                    serialVal = SERVOSTOPPULSE;
                }

                servo_write_pulse(serialVal);

            }else{
                overflow=false;
            }

            i = 0;
            for (uint8_t j=7; j; j--){
                strSerialValue[j]=0;
            }
        }
    }
}

//==========================================================================
// PORT1 VECTOR ISR
// funcao: servico de interrupcao PORT1. Captura valores dos timers e botao
// retorno: nenhum
// parametros: nenhum
// constantes:
//      BUTTONPIN: botao
//      MOTORINPIN: saida do filtro
//==========================================================================
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
        buttonFlag = false;
        // limpa flag de interrupcao
        P1IFG &= ~BUTTONPIN;
    }else if(P1IFG & MOTORINPIN){
        // captura tempo atual
        overTimer = timerOverflow;
        timerCount = TA1R;
        // zera os contadores
        timerOverflow = TA1R = 0;
        // limpa flag de interrupcao
        P1IFG &= ~MOTORINPIN;
    }
}

//==========================================================================
// TIMER 0 A0
// funcao: servico de interrupcao TIMER 0 A0. Limpa IFG
// retorno: nenhum
// parametros: nenhum
// constantes: nenhuma
//==========================================================================
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

//==========================================================================
// TIMER 1 A0
// funcao: servico de interrupcao TIMER 1 A0. Incrementa estouro do timer
// retorno: nenhum
// parametros: nenhum
// constantes: nenhuma
//==========================================================================
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

//==========================================================================
// TIMER 0 A1
// funcao: servico de interrupcao TIMER 0 A1. Atualiza PWM (TA0CCR1),
//         prox. amostragem (TA0CCR2), gatilho para amostra
// retorno: nenhum
// parametros: nenhum
// constantes:
//      SAMPLINGINTERVAL: tempo de amostragem
//==========================================================================
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) Timer0_A1 (void)
#else
#error Compiler not supported!
#endif
{
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)    
    switch(__even_in_range(TA0IV, TA0IV_TAIFG)){
#else
    switch(TA0IV){
#endif
//        case TA0IV_NONE: break;
//        case TA0IV_TACCR1: break;
        case TA0IV_TACCR2:
            amostrar = true; // habilita envio
            TA0CCR2 += SAMPLINGINTERVAL; // prox. envio
            break;
//        case TA0IV_6: break;
//        case TA0IV_8: break;
        case TA0IV_TAIFG:
            amostrar = true; // habilita envio
            TA0CCR1 = nextPulse; // atualiza pwm
            TA0CCR2 = SAMPLINGINTERVAL; // prox.envio
            break;
        default:
//            amostrar = true; // habilita amostragem
//            TA0CCR2 += SAMPLINGINTERVAL; // prox. envio
            break;
    }
}

//==========================================================================
// CLOCK CONFIG
// funcao: configura clock principal @16MHz
// retorno: nenhum
// parametros: nenhum
// constantes: nenhuma
//==========================================================================
void clock_config(){
    if (0xFF == CALBC1_16MHZ){
        while(1);
    }

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;
}

//==========================================================================
// SERVO CONFIG
// funcao: configura PWM @50Hz
// retorno: nenhum
// parametros: duty cycle, em ms (uint16_t)
// constantes: nenhuma
//==========================================================================
void servo_config(){
    TA0CTL |= TASSEL_2 | ID_3 | MC_1 | TAIE; // SMCLK, DIV(8), UP CCR0, INT
//    TA0CCTL0 |= CCIE; //INT
    TA0CCR0 = 40000-1; // periodo (16Mhz/8/1000ms) * 20ms = 40_000 ciclos
    TA0CCTL1 |= OUTMOD_7; // reset/set
    TA0CCR1 = 0; // reset
}

//==========================================================================
// SERVO WRITE PULSE
// funcao: aplica o valor de pulso para o prox. PWM
// retorno: nenhum
// parametros: duty cycle, em ms (uint16_t)
// constantes:
//      SERVOMAXPULSE: limite superior
//      SERVOMINPULSE: limite inferior
//==========================================================================
inline void servo_write_pulse(uint16_t ms){
    // limita o pulso
    // if(ms > SERVOMAXPULSE){
    //     ms = SERVOMAXPULSE;
    // }else if(nextPulse < SERVOSTOPPULSE){
    //     ms = SERVOSTOPPULSE;
    // }

    nextPulse = (ms<<1)-1; // prox. pwm
}

//==========================================================================
// GPIO CONFIG
// funcao: configura pinos entrada/saida/funcao
// retorno: nenhum
// parametros: nenhum
// constantes:
//      MOTOROUTPIN: entrada do filtro do motor
//      MOTORINPIN: saida PWM para o ESC
//      REDLEDPIN: led vermelho (sinalizacao)
//      BUTTONPIN: botao
//==========================================================================
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

//==========================================================================
// CRONOMETRO CONFIG
// funcao: configura TA1CCR0 para estourar a cada 3,125ms (calc. velocidade)
// retorno: nenhum
// parametros: nenhum
// constantes: nenhuma
//==========================================================================
void cronometro_config(){
    // configura o timer A1
    TA1CTL = TASSEL_2 | ID_0 | MC_1; // smclk, div 1, up CCR0
    TA1CCTL0 |= CCIE; // interrupcao por comparacao
    TA1CCR0 = 50000-1; // @16MHz: 1/320 = 3,125ms para estourar
}

//==========================================================================
// SAMPLING CONFIG
// funcao: configura o tempo de amostragem (TA0CCR2)
// retorno: nenhum
// parametros: nenhum
// constantes: SAMPLINGINTERVAL (2*tempo_de_amostragem-1) (em ms)
//==========================================================================
void sampling_config(){
    TA0CCTL2 |= CCIE; //INT
    TA0CCR2 = SAMPLINGINTERVAL; // tempo de amostragem
}

//==========================================================================
// ITOA BASE 10
// funcao: converte um numero inteiro para c_string
// retorno: nenhum
// parametros: numero (uint32_t), string (char*)
// constantes: nenhuma
//==========================================================================
void itoa_base_10(int32_t num, char* str){

    char* ptr = str, *ptr1 = str, char_tmp;
    int int_tmp;

    do{
        int_tmp = num;
        num /= 10;
        const char charTable[] = "zyxwvutsrqponmlkjihgfedcba9876543210"
                                 "123456789abcdefghijklmnopqrstuvwxyz";
        *ptr++ = charTable[35 + (int_tmp - num * 10)];
    }while(num);

    if(int_tmp < 0) *ptr++ = '-';

    *ptr-- = '\0';

    while(ptr1 < ptr){
        char_tmp = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = char_tmp;
    }
}

//==========================================================================
// DELAY MS
// funcao: pausa o programa
// retorno: nenhum
// parametros: tempo de espera, em ms (uint16_t)
// constantes: nenhuma
//==========================================================================
void delay_ms(uint16_t ms){
    while (ms--) {
        __delay_cycles(16000);
    }
}
