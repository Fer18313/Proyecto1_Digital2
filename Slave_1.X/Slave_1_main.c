/*
 * File:   main.c
 * Author: Fernando Sandoval
 * Carne: 18313
 * Created on August 31, 2021, 10:02 PM
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)
// LIBRERIAS
#include <xc.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <pic16f887.h>
#include "LCD.h"
#include "I2C.h"
#define _XTAL_FREQ 8000000


// VARIABLES
unsigned int sun_pot=0;
float an_sun=0;
unsigned long LDR=0;
unsigned char buffer[16];
//uint8_t Unit, dec0, dec1;

// PROTOTYPE FUNCTIONS
void initSETUP(void);
void str_2_dec(uint16_t var);

void main(void) {
    initSETUP();
    Lcd_Init();                     
    Lcd_Clear();  
    Lcd_Set_Cursor(1,1); 
    Lcd_Write_String(" S1:   S2:   S3:");
    ADCON0bits.GO = 1;
    while(1){
        an_sun = (48.8758*sun_pot)/(5.0-(0.00488758*sun_pot));
        LDR = an_sun;
        sprintf(buffer,"LCD: %6u",LDR);
        Lcd_Set_Cursor(2,1);
        Lcd_Write_Char(an_sun);
        __delay_ms(500);
    }
    return;
}

void __interrupt()isr(void){
    if (ADIF == 1){                            
        sun_pot = ADRESH;
        sun_pot = sun_pot <<8;
        sun_pot |= ADRESL;
        ADIF = 0; //Limpiar la bandera de ADC
        __delay_us(50);
        ADCON0bits.GO = 1; //Inicia la conversión de ADC
    }
}
void initSETUP(void){
    TRISA = 0b00000001;
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    PORTE = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    ANSEL = 0b00000001;
    ANSELH = 0;
    OSCCONbits.IRCF2 = 1; 
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    //ADC CONFIG
    ADCON1bits.ADFM = 0; //Justificar a la izquierda
    ADCON1bits.VCFG0 = 0; //Vss
    ADCON1bits.VCFG1 = 0; //VDD
    ADCON0bits.ADCS = 0b10; //ADC oscilador -> Fosc/32
    ADCON0bits.CHS = 0;     //Comenzar en canal 0       
    ADCON0bits.ADON = 1;    //Habilitar la conversión ADC
    __delay_us(50); 
    ADCON0bits.GO = 1;
    // MAIN INTERRUPTIONS
    INTCONbits.GIE = 1;
    INTCONbits.PEIE =1;
    return;
}