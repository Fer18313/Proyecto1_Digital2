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

// CRYSTAL DEL PIC
#define _XTAL_FREQ 8000000

// FUNCTION PROTOTYPES
void initSETUP(void);
void startRead(void);
void checkResponse(void);
char DHT11_Read();


// FUNCTION VARIABLES
uint8_t buffer;
uint8_t test =0;
unsigned char RH_Dec;
unsigned char RH_Int;
unsigned char T_Dec;
unsigned char T_Int;
unsigned char Humidity, RH;
unsigned char check_sum;
uint8_t unit0, dec0, unit1, dec1;

// EL SENSOR DHT11 NECESITA QUE NOSOTROS LO INICIEMOS, PARA ELLO
// UTILIZAMOS startRead()

// ADEMAS NECESITAMOS CHEQUEAR SI TENEMOS RESPUESTA DEL MISMO
// UTILIZAMOS checkResponse()

// FINALMENTE EXTRAEMOS DATA DEL PIN
// UTILIZAMOS DHT11_Read()



void main(void) {
    initSETUP();                // CONFIGURACION DEL PIC ESCLAVO
    while (1){
        startRead();        // INICIAMOS EL SENSOR
        checkResponse();    // CHEQUEAMOS SI RECIBIMOS ALGO
        RH_Int = DHT11_Read();  // PRIMER BYTE
        RH_Dec = DHT11_Read();  // SEGUNDO BYTE
        T_Int = DHT11_Read();   // TERCER BYTE
        T_Dec = DHT11_Read();   // CUARTO BYTE
        check_sum = DHT11_Read();   // QUINTO BYTE| PARA ESTA APLICACION ESTA INUTILIZADO.
        
        RH = T_Int;     // GUARDAMOS EL VALOR DE TEMP   
        Humidity = RH_Int;  // GUARDAMOS EL VALOR DE HUMEDAD RELATIVA
        __delay_ms(1000);
    }
    return;
}
void __interrupt()isr(void){ // INTERRUPCION PARA COMUNICARSE POR MEDIO DE I2C
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
            if (test==0){                   // PARA ENVIAR AMBOS VALORES, UTILIZAMOS POLLING 
                SSPBUF = RH;                // SUBIMOS AL BUFFER DATO DE TEMPERATURA
                test = 1;
            }
                else if (test==1){
                SSPBUF = Humidity;          // SUBIMOS AL BUFFER DATO DE HUMEDAD RELATIVA
                test = 0;
                }
            SSPCONbits.CKP = 1; // SCL PULSE ON
            __delay_us(250);
            while(SSPSTATbits.BF);
        }
        PIR1bits.SSPIF = 0;    
    }
}

void initSETUP(void){
    TRISA = 0;
    TRISB = 0;
    TRISC = 0; 
    TRISD = 0;
    TRISE = 0;
    PORTE = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    ANSEL = 0;
    ANSELH = 0;
    OSCCONbits.IRCF2 = 1; 
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    // MAIN INTERRUPTIONS
    INTCONbits.GIE = 1;
    INTCONbits.PEIE =1;
    I2C_Slave_Init(0x80);   // DIRECCION DEL SLAVE PARA COMUNICARSE POR I2C
    return;
}


void startRead(){               // RUTINA PARA INICIAR EL DHT11
    TRISA = 0b00000010;     // OFF 
    PORTAbits.RA0 = 0;
    __delay_ms(18);         // PULSO APAGADO DE 18ms, ES NECESARIO SEGUN DOCUMENTACION DEL DHT11
    PORTAbits.RA0 = 1;
    __delay_us(30);         // FINALMENTE UN PULSO ENTRE 20-60 us
    TRISA= 0b00000011;      // SET COMO ENTRADA EL PIN AN0;
}

void checkResponse(){           // NO NOS MOVEMOS HASTA OBTENER DATOS DEL SENSOR
    while (PORTAbits.RA0 & 1);
    while (!(PORTAbits.RA0 & 1));
    while (PORTAbits.RA0 & 1);
}

char DHT11_Read(){              // AQUI HACEMOS EL SHIFT DE LOS BITS PARA OBTENER
    char i,data = 0;
    for (i=0; i<8; i++){
        while(!(PORTAbits.RA0 & 1));
        __delay_us(30);
        if(PORTAbits.RA0 & 1)
            data = ((data<<1) | 1);
        else
            data = (data<<1);
        while(PORTAbits.RA0 & 1);
}
    return data;
}
