// TE149-motor-brushless
// Controlador PID Digital para motor brushless
// Disciplina TE149 - Instrumentação Eletrônica - UFPR 2016/1
// https://github.com/wsilverio/TE149-motor-brushless
//
// compilado com "msp430-gcc", C99 para mcu=msp430g2553

#include <msp430.h>     // PxOUT, PxIN, ...
#include <stdint.h>     // int8_t, int16_t, ...
#include <stdbool.h>    // bool
#include <stdlib.h>     // itoa
#include <string.h>     // ftoa

// #define SERIAL_DBG

#define LEDPIN          BIT0    // P1.0 (RED LED)
#define BUTTONPIN       BIT3    // P1.3 (S2)
#define SERIALRXPIN     BIT1    // P1.1
#define SERIALTXPIN     BIT2    // P1.2
#define MOTORINPIN      BIT4    // P1.4
#define MOTOROUTPIN     BIT6    // P1.6 / TA01 / GREEN LED

#define MOTORPOLES      7.0
#define SERVOSTOPPULSE  1000    // 1ms
#define SERVOMAXPULSE   2000    // 2ms
// #define RPMMAX          6000
// #define RPMMIN          2000

#define SERIALPRINTTIME 250

#define NM              20.0        // numero de medias
#define ALPHA           NM/(NM+1)   // coeficiente exponencial

// void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
void serial_config();
void serial_print_byte(const int8_t data);
void serial_print_string(const char* data);
void servo_config();
void servo_write_pulse(const uint16_t ms);
// uint16_t servo_get_pulse();
void ftoa(float n, char *res, int casas);
float _pow(float base, float expoente);

volatile uint16_t servoPulse = 0;

volatile bool loopFlag = false;
volatile bool stopFlag = false;
// volatile bool buttonFlag = false;

volatile uint16_t rpm[2];
volatile uint16_t flagTimer = 0;

char strRpmValue[8];    // string de uso geral
char strSerialValue[8]; // string de uso geral

void main(){
    // desabilita watchdog
    WDTCTL = WDTPW + WDTHOLD;

    // config. clock 16Mhz
    if (0xFF == CALBC1_16MHZ){
        while(1);
    }

    DCOCTL = 0;
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;

    // config. I/O
    P1DIR |= (MOTOROUTPIN | LEDPIN);    // saidas
    P1SEL |= (MOTOROUTPIN);             // pwm out
    P1REN |= (MOTORINPIN | BUTTONPIN);  // habilita resistores
    P1OUT |= (MOTORINPIN | BUTTONPIN);  // resistores pullup

    P1OUT &= ~(LEDPIN);   // limpa as saidas

    // config. perifericos
    serial_config();
    servo_config();

    // zera as medidas
    rpm[0] = rpm[1] = 0;

    // configura o timer A1
    TA1CTL    = TASSEL_2 | ID_0 | MC_1; // smclk, div 1, up CCR0
    TA1CCTL0 |= CCIE;                   // interrupcao por comparacao
    TA1CCR0   = 50000-1;                // @16MHz -> 1/320 = 3,125ms para estourar

    // configura interrupcao motor
    P1IE  |= (MOTORINPIN | BUTTONPIN);  // habilita interrupcao
    P1IES |= (MOTORINPIN | BUTTONPIN);  // borda de descida
    P1IFG  = 0;             // limpa IFG

    __enable_interrupt();

    servo_write_pulse(SERVOSTOPPULSE);

    loopFlag = true;

    while(1){
        if(!stopFlag){
            P1OUT ^= LEDPIN;
            delay_ms(SERIALPRINTTIME);
            itoa(rpm[1], strRpmValue, 10);
            serial_print_string(strRpmValue);
            serial_print_byte('\n');
        }
    }
}

#pragma vector = PORT1_VECTOR
__interrupt void interrupt_port_1(){
    if (P1IFG & MOTORINPIN){

        // captura tempo atual
        uint16_t overTimer = flagTimer;
        uint16_t timerCount = TA1R;
        float delta_t = (62.5e-9*timerCount + 3.125e-3*overTimer);
        flagTimer = TA1R = 0;

        // conversao para RPM
        uint16_t rpmInst = (uint16_t)((60.0/MOTORPOLES)/delta_t);
        
        rpm[1] = ALPHA*rpm[0]+(1-ALPHA)*rpmInst;
        rpm[0] = rpm[1];
        
        // limpa flag de interrupcao
        P1IFG &= ~MOTORINPIN;

    }else if (P1IFG & BUTTONPIN){

        P1OUT ^= LEDPIN;

        if(loopFlag){

            stopFlag = !stopFlag;

            if(stopFlag){
                TA0CTL &= ~MC_0;    // desliga o timer
                P1OUT &= ~LEDPIN;   // apaga o led
            }else{
                TA0CTL |= MC_1;     // liga o timer
            }
        }

        // debouncing
        delay_ms(10);
        while(!(P1IN & BUTTONPIN));
        delay_ms(10);

        // limpa flag de interrupcao
        P1IFG &= ~BUTTONPIN;
    }
}

#pragma vector = TIMER1_A0_VECTOR
__interrupt void interrupt_timer_A1(){

    ++flagTimer;

    TA1CCTL0 &= ~CCIFG;
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void serial_receive(){

    static uint8_t i = 0;

    static bool overflow = false;

    char val = UCA0RXBUF;

    if (7==i && '\n' != val){

        overflow = true;

#ifdef SERIAL_DBG
        serial_print_string("\n*** buffer overflow ***\n");
#endif        
        
        for (uint8_t j=0; j<8; j++){
            strSerialValue[j]=0;
        }

        i = 0;
    }else if(7 >= i){
        
        strSerialValue[i] = val;
        i++;

        if ('\n' == val){
            if(!overflow){
                strSerialValue[i-1] = '\0';
                int pulseWidth = atoi(strSerialValue);
                if(SERVOSTOPPULSE <= pulseWidth && SERVOMAXPULSE >= pulseWidth ){
                    servo_write_pulse(pulseWidth);
                }
            }else{
                overflow=false;
            }
        
            i = 0;
            for (uint8_t j=0; j<8; j++){
                strSerialValue[j]=0;
            }
        }
    }
}

void servo_config(){
    TA0CTL = TASSEL_2 | ID_3 | MC_1;    // SMCLK, DIV(8), UP CCR0
    TA0CCTL1 |= OUTMOD_7;               // PWM set/reset
    TA0CCR0 = 20000-1;                  // periodo: (16Mhz / 8 / 1000ms) * 20ms
    TA0CCR1 = 0;                        // reset
}

void servo_write_pulse(const uint16_t ms){
    TA0CCR1 = 2*ms;
}

// uint16_t servo_get_pulse(){
//     return TA0CCR1;
// }

void serial_config(){
    P1SEL  |= (SERIALRXPIN | SERIALTXPIN);
    P1SEL2 |= (SERIALRXPIN | SERIALTXPIN);

    UCA0CTL1 |= UCSWRST;
    UCA0CTL0 &= ~(UCMODE1 + UCMODE0 + UCSYNC);
    UCA0CTL1 |= UCSSEL_2;
    UCA0MCTL &= ~UCOS16;

    // 115200 bps
    UCA0BR1 = 0x00;
    UCA0BR0 = 0x8A;
    UCA0MCTL |= 0x0E;

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

void delay_ms(uint16_t ms){
    while (ms--) {
        __delay_cycles(16000);
    }
}

/*void delay_us(uint16_t us){
    while(us--){
        __delay_cycles(16);
    }
}*/

#ifdef SERIAL_DBG
void ftoa(float n, char *res, int casas){
    // parte inteira do número
    int iparte = (int) n;
    // parte flutuante (após a vírgula)
    float fparte = n - (float) iparte;
    // carrega a parte inteira na string
    itoa(iparte, res, 10);
    if (casas != 0){
        // adiciona o 'ponto'
        strcat(res, ".");
        // tranforma a parte flutuante
        fparte = fparte * f_pow(10, casas);
        // concatena a parte flutuante à parte inteira
        itoa((int)fparte, res + strlen(res), 10);
    }
}

float f_pow(float base, float expoente){
    float n = base;
    while(--expoente) n *= base;
    return n;
}
#endif
