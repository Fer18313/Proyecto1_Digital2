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
#include <pic16f887.h>
#include "LCD.h"
#include "I2C.h"
#define _XTAL_FREQ 8000000

// FUNCTION PROTOTYPES
void initSETUP(void);

// VARIABLES
uint8_t unit0, dec0, unit1, dec1;
unsigned char Humidity=0;
unsigned char RH1;

void main(void) {
    initSETUP();
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String(" S1:   S2:   S3:");
    while(1){
        I2C_Master_Start();         //Se inicializa la comunicacion I2C
        I2C_Master_Write(0x81);     //first slave
        RH1 = I2C_Master_Read(0); 
        I2C_Master_Stop();          //Termina la comunicacion 
        __delay_ms(200);
        Lcd_Set_Cursor(2,1);             
        Lcd_Write_Char(unit0);
        Lcd_Write_Char(dec0);
        Lcd_Write_String("%   ");
        Lcd_Write_Char(unit1);
        Lcd_Write_Char(dec1);
        Lcd_Write_Char("°");
        Lcd_Write_String("C  "); 
        Lcd_Write_String("000");
        __delay_ms(1000);
    }
    return;
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
    I2C_Master_Init(100000);
    return;
}