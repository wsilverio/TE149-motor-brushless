// TE149-motor-brushless
// Controlador PID Digital para motor brushless
// Disciplina TE149 - Instrumentação Eletrônica - UFPR 2016/1
// https://github.com/wsilverio/TE149-motor-brushless
// 
// compilado com "msp430-gcc", C99 para mcu=msp430g2553

#include <msp430.h>     // PxOUT, PxIN, ...
#include <stdint.h>     // int8_t, int16_t, ...
#include <stdbool.h>    // bool

#define LEDPIN          BIT0    // P1.0 (RED LED)
#define BUTTONPIN       BIT3    // P1.3 (S2)
#define SERIALRXPIN     BIT1    // P1.1
#define SERIALTXPIN     BIT2    // P1.2
#define MOTORINPIN      BIT4    // P1.4
#define MOTOROUTPIN     BIT6    // P1.6 / TA01 / GREEN LED

#define MOTORPOLES      7
#define SERVOMINPULSE   500     // 0.5ms
#define SERVOMAXPULSE   2500    // 2.5ms
#define SERVOMINDEGREE  0
#define SERVOMAXDEGREE  179
#define RPMMAX          5000
#define RPMMIN          2000

void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
inline uint32_t millis();
void itoa(long unsigned int value, char* result);
void serial_config();
void serial_print_byte(int8_t data);
void serial_print_string(char* data);
void servo_config();
void servo_write_degree(uint8_t degree);
void servo_write_pulse(uint16_t ms);
uint16_t degree_to_ms(uint8_t degree);

volatile uint16_t servoPulse = 0;
volatile uint32_t _millis_ = 0;

volatile bool buttonFlag = false;
volatile bool stopFlag = false;
volatile bool loopFlag = false;

volatile int16_t rpm = -1;
volatile uint32_t lastSerial = 0;

char strValue[10]; // string de uso geral

void main(){
    // desabilita watchdog
    WDTCTL = WDTPW + WDTHOLD;

    // clock @8MHz
    BCSCTL1 = CALBC1_8MHZ;
    DCOCTL = CALDCO_8MHZ;

    P1DIR |= (MOTOROUTPIN | LEDPIN);    // saidas
    P1SEL |= (MOTOROUTPIN);             // pwm out
    P1REN |= (MOTORINPIN | BUTTONPIN);  // habilita resistores
    P1OUT |= (MOTORINPIN | BUTTONPIN);  // resistores pullup

    P1OUT &= ~(LEDPIN | MOTOROUTPIN);   // limpa as saidas

    // config. perifericos
    serial_config();
    servo_config();

    // configura o timer A1
    TA1CTL    = TASSEL_2 | ID_3 | MC_1; // smclk, div 8, up CCR0
    TA1CCTL0 |= CCIE;                   // interrupcao por comparacao
    TA1CCR0   = 1000-1;                 // @8MHz -> 1ms

    // configura interrupcao motor
    P1IE  |= (MOTORINPIN);  // habilita interrupcao
    P1IES |= (MOTORINPIN);  // borda de descida
    P1IFG  = 0;             // limpa IFG    

    __enable_interrupt();

    // calibacao brushless
    if (!(P1IN & BUTTONPIN)){
        // sinaliza modo configuracao (pisca 3x)
        for(uint8_t i = 0; i < 3; ++i){
            P1OUT |= LEDPIN;
            delay_ms(50);
            P1OUT &= ~LEDPIN;
            delay_ms(250);
        }

        // aguarda botao livre
        while(!(P1IN & BUTTONPIN));
        // espera 1s
        delay_ms(1000);

        // configura interrupcao botao
        P1IE  |= (BUTTONPIN);  // habilita interrupcao
        P1IES |= (BUTTONPIN);  // borda de descida
        P1IFG &= ~(BUTTONPIN); // limpa IFG           

        P1OUT |= LEDPIN;
        // aguarda usuario
        buttonFlag = false;
        while(!buttonFlag);

        P1OUT &= ~LEDPIN;

        // pulso maximo por 2s
        servo_write_pulse(SERVOMAXPULSE);
        delay_ms(2000);

        P1OUT |= LEDPIN;
        // aguarda usuario
        buttonFlag = false;
        while(!buttonFlag);

        P1OUT &= ~LEDPIN;

        // pulso minimo por 2s
        servo_write_pulse(SERVOMINPULSE);
        delay_ms(2000);

    }else{
        // configura interrupcao botao
        P1IE  |= (BUTTONPIN);  // habilita interrupcao
        P1IES |= (BUTTONPIN);  // borda de descida
        P1IFG &= ~(BUTTONPIN); // limpa IFG 
    }

    P1OUT |= LEDPIN;
    // aguarda comando de inicio
    buttonFlag = false;
    while(!buttonFlag);
    // apaga o led
    P1OUT &= ~LEDPIN;
    
    buttonFlag = false;
    loopFlag = true;

    while(1){
        if (!stopFlag){
            for (int i = SERVOMINPULSE; i <= SERVOMAXPULSE && !stopFlag ; i+=5){
                servo_write_pulse(i);
                delay_ms(3);
            }
            for (int i = SERVOMAXPULSE; i >= SERVOMINPULSE && !stopFlag; i-=5){
                servo_write_pulse(i);
                delay_ms(3);
            }            
        }
    }
}

#pragma vector = PORT1_VECTOR
__interrupt void interrupt_port_1(){
    if (P1IFG & MOTORINPIN){

        static uint32_t lastTime = 0;
        // captura tempo atual
        uint32_t timeNow = millis();

        // conversao para RPM
        rpm = (int16_t)(1000.0/(timeNow - lastTime) / MOTORPOLES * 60);
        lastTime = timeNow;

        // if (rpm < RPMMIN || rpm > RPMMAX){
        //     rpm = -1;
        // }

        // envia a velocidade pela serial
        if (timeNow - lastSerial > 1000){
            itoa(rpm, strValue);
            serial_print_string(strValue);
            lastSerial = timeNow;
        }

        // limpa flag de interrupcao
        P1IFG &= ~MOTORINPIN;

    }else if (P1IFG & BUTTONPIN){

        buttonFlag = true;

        if (loopFlag){
            stopFlag = !stopFlag;
            servo_write_pulse(0);
            P1OUT = (stopFlag)?(P1OUT | LEDPIN):(P1OUT & ~LEDPIN);
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

    ++_millis_;

    TA1CCTL0 &= ~CCIFG;
}

inline uint32_t millis(){
    return _millis_;
}

void servo_config(){
    TA0CTL = TASSEL_2 | ID_3 | MC_1;    // SMCLK, DIV(8), UP CCR0
    TA0CCTL1 |= OUTMOD_7;               // PWM set/reset
    TA0CCR0 = 20000-1;                  // periodo: (8Mhz / 8 / 1000ms) * 20ms
    TA0CCR1 = 0;                        // reset
}

void servo_write_pulse(uint16_t ms){
    // 
    TA0CCR1 = ms;
}

void servo_write_degree(uint8_t degree){
    servo_write_pulse( degree_to_ms(degree) );
}

uint16_t degree_to_ms(uint8_t degree){
    if(degree > SERVOMAXDEGREE){
        degree = SERVOMAXDEGREE;
    }else if(degree < SERVOMINDEGREE){
        degree = SERVOMINDEGREE;
    }

    return (uint16_t)(SERVOMINPULSE + (SERVOMAXPULSE - SERVOMINPULSE) * ((float)degree / SERVOMAXDEGREE));
}

void serial_config(){
    // configura pinos
    P1SEL |= (SERIALRXPIN | SERIALTXPIN);
    P1SEL2 |= (SERIALRXPIN | SERIALTXPIN);
    // USCI reset: desabilitado para operacao
    // UCSWRST (BIT1) = 1
    UCA0CTL1 |= UCSWRST; // |= 0x01
    // modo UART:
    //      UCMODE1 (BIT2) = 0
    //      UCMODE0 (BIT1) = 0
    // modo assincrono:
    //      UCSYNC (BIT0) = 0
    UCA0CTL0 &= ~(UCMODE1 + UCMODE0 + UCSYNC); // &= ~0x07
    // USCI clock: modo 2 (SMCLK):
    // UCSSEL (BIT7 e BIT6):
    //      (BIT7) = 1
    //      (BIT6) = 0
    UCA0CTL1 |= UCSSEL_2; // |= 0x80
    // Oversampling desabilitado
    //      UCOS16 (BIT1) = 0
    UCA0MCTL &= ~UCOS16;
    // 9600 bps: 8M / 9600 = 833,333
    // int(8M / 9600) = 833 = 0x[03][41]
    // round((833,333 - 833)*8) = [3]
    UCA0BR1 = 0x03;
    UCA0BR0 = 0x41;
    UCA0MCTL |= 0x06; // |= (0x[3]) << 1;
    // USCI reset: liberado para operacao
    UCA0CTL1 &= ~UCSWRST; // &= ~0x01
    // Habilita interrupcao de recepcao
    IE2 |= UCA0RXIE;
}

void serial_print_byte(int8_t data){
    // aguarda buffer vazio
    while(!(IFG2 & UCA0TXIFG));
    // escreve o dado no registrador
    UCA0TXBUF = data;
}

void serial_print_string(char* data){
    while(*data){
        serial_print_byte(*data);
        data++;
    }
}

void delay_ms(uint16_t ms){
    while (ms--) {
        __delay_cycles(8000);
    }
}

void delay_us(uint16_t us){
    while(us--){
        __delay_cycles(8);
    }
}

void itoa(long unsigned int value, char* result){
    char* ptr = result, *ptr1 = result;
    int tmp_value;

    do {
        tmp_value = value;
        value /= 10;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz" [35 + (tmp_value - value * 10)];
    } while ( value );

    // Apply negative sign
    if (tmp_value < 0) *ptr++ = '-';
    
    *ptr-- = '\0';
    while(ptr1 < ptr) {
        char* tmp_char;
        tmp_char = *ptr;
        *ptr--= *ptr1;
        *ptr1++ = tmp_char;
    }
}
