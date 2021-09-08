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
uint16_t ADCread=0;         // VALOR DEL ADC OBTENIDO DEL CANAL 0/AN0
float supply;
int LDR;
uint8_t vcv,unit0,dec0,dec1,buffer;

//PROTOTYPE FUNCTIONS
void initSETUP(void);
void str_2_dc(uint16_t var);

void main(void) {
    initSETUP();
    ADCON0bits.GO = 1; // LISTOS PARA REALIZAR CONVERSION
    while(1){
        supply = 5.0*ADCread/1024.0;
        LDR = (100*supply)/1.185; // REALIZAMOS LA CONVERSION DEL RANGO PARA UNO DE 0-100%
        __delay_ms(50);
    }
    return;
}

void __interrupt()isr(void){        // INTERRUPCION PARA ENVIAR POR MEDIO DE I2C
    if (PIR1bits.ADIF == 1){                            
        ADCread = ADRESH;
        PIR1bits.ADIF = 0; 
        __delay_us(60);
        ADCON0bits.GO = 1;
    }
    if(PIR1bits.SSPIF == 1){ 
        SSPCONbits.CKP = 0;     
        if ((SSPCONbits.SSPOV) || (SSPCONbits.WCOL)){
            buffer = SSPBUF;    
            SSPCONbits.SSPOV = 0;       
            SSPCONbits.WCOL = 0;        
            SSPCONbits.CKP = 1;         
        }
        if(!SSPSTATbits.D_nA && !SSPSTATbits.R_nW) {
            buffer = SSPBUF;     
            PIR1bits.SSPIF = 0;         
            SSPCONbits.CKP = 1;         
            while(!SSPSTATbits.BF);     
            __delay_us(250);
        }else if(!SSPSTATbits.D_nA && SSPSTATbits.R_nW){
            buffer = SSPBUF; 
            BF = 0;
            SSPBUF = LDR;           // ENVIAMOS EL VALOR DE LA LUMINOSIDAD EN SU FORMATO DE PERCENTIL
            SSPCONbits.CKP = 1;
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
        PIR1bits.SSPIF = 0;    
    }
}

void str_2_dc(uint16_t var){        // FUNCION PARA CONVERTIR LOS DATOS EN ASCII
    uint16_t vcv;
    vcv = var;                  
    unit0 = (vcv/100) ;               
    vcv = (vcv - (unit0*100));      // CONVERSION A DEC
    dec0 = (vcv/10);             
    vcv = (vcv - (dec0*10));
    dec1 = (vcv);                
    
    unit0 = unit0 + 48;  // ASCII         
    dec0 = dec0 + 48;
    dec1 = dec1 + 48;
    
}
void initSETUP(void){
    TRISA = 0b00000001; //RA0
    TRISB = 0;
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    PORTE = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    ANSEL = 0b00000001; // AN0
    ANSELH = 0;
    
    // CONFIGURACION DEL ADC Y DE SU INTERRUPCION
    
    ADCON1bits.ADFM = 0; 
    ADCON1bits.VCFG0 = 0; //Vss
    ADCON1bits.VCFG1 = 0; //VDD

    ADCON0bits.ADCS = 0b10; // Fosc/32
    ADCON0bits.CHS = 0;     // CHANNEL 0/AN0      
    ADCON0bits.ADON = 1;    // GO CONVERT...
    __delay_us(50); 
    ADCON0bits.GO = 1;
    PIE1bits.ADIE = 1;   // ADC INTERRUPT ENABLE
    PIR1bits.ADIF = 0;   // LIMPIAMOS LA BANDERA MANUALMENTE PARA EVITAR CUALQUIER MALA TOMA
    OSCCONbits.IRCF2 = 1; 
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    
    // MAIN INTERRUPTIONS
    INTCONbits.GIE = 1;
    INTCONbits.PEIE =1;
    I2C_Slave_Init(0x60); // DIRECCION DEL ESCLAVO PARA COMUNICARSE CON EL MASTER
    return;
}