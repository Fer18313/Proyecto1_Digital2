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
#include "USART.h"
#define _XTAL_FREQ 8000000

// FUNCTION PROTOTYPES
void initSETUP(void);
void str_2_dc(uint16_t var);


// VARIABLES
uint8_t unit0, dec0, unit1, dec1, unit0_0, dec0_12,dec1_12;
unsigned char Humidity=0;
unsigned char RH=0;
int LDR = 0;
uint8_t test =0;
uint8_t cont = 0,cont1 = 0;
uint8_t F1=0; 
uint8_t counter, tempRX;


void main(void) {
    initSETUP();
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
    Lcd_Write_String(" RH:   T:   L%:");
    while(1){
        I2C_Master_Start();         // UTILIZAMOS I2C PARA OBTENER LOS DATOS 
        I2C_Master_Write(0x81);     // PRIMER ESCLAVO
        if (test==0){                           // HACEMOS POLLING PARA SABER QUE DATO NOS ENTRA
                RH=I2C_Master_Read(0);
                test = 1;
            }
                else if (test==1){
                Humidity=I2C_Master_Read(0);
                test = 0;
            }
        I2C_Master_Stop();          //TERMINAMOS
        __delay_ms(200);
        I2C_Master_Start();         // EMPEZAMOS SEGUNDA COM I2C
        I2C_Master_Write(0x61);     // ADDRESS DEL SEGUNDO PIC
        LDR = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
        Lcd_Set_Cursor(2,1);             
                                                //CONVERSIONES ASCII PARA MOSTRAR EN LCD
        unit0 = 48 + ((Humidity/10) %10);
        dec0 = 48 + (Humidity %10);
        unit1 =48 + ((RH / 10) % 10);
        dec1 = 48 + (RH % 10);
          
        
        Lcd_Write_Char(unit0);              //DESPLEGAMOS EN LCD
        Lcd_Write_Char(dec0);
        Lcd_Write_String("%   ");
        Lcd_Write_Char(unit1);
        Lcd_Write_Char(dec1);
        Lcd_Write_Char(223);
        Lcd_Write_String("C  ");
        
        str_2_dc(LDR);                      // CONVERTIMOS EN ASCII
        Lcd_Write_Char(unit0_0);            // DESPLEGAMOS EN LCD
        Lcd_Write_Char(dec0_12);
        Lcd_Write_Char(dec1_12);
        Lcd_Write_String("%");
        
        // CONTROL PARA LOS MOTORES
        
        if(RH>34 && cont ==1){
            //Do something
            PORTAbits.RA3 = 1;          // 5 V PARA HABILITAR LA BOMBA
            __delay_ms(3000);           // LUEGO DE UN TIEMPO REGRESA A 0 V.
            PORTAbits.RA3 = 0;          
            cont =0;                    // con este contador evitamos que se sirva infinitamente.
        }
        else if(RH<35 && cont ==0){
            PORTAbits.RA3 = 0;          // TODO EL TIEMPO ESTAMOS EN 0 V 
            cont = 1;                            // SI NO SE CUMPLE LA CONDICION.
            //Do something
        }
        else if (20<LDR<=80 && cont1 ==0){
            // PONEMOS EN POSICION CERRADA EL SERVO PARA SERVIR COMIDA
            PORTAbits.RA0 = 1;
            __delay_us(2000);
            PORTAbits.RA0 = 0;
            __delay_ms(1000);
            cont1 = 1;
       // Lcd_Write_String("000");    
        }
        else if (LDR<=20 && cont1==1){
            // PONEMOS EN POSICION Abierta EL SERVO PARA SERVIR COMIDA
            cont1 = 0;
            PORTAbits.RA0 = 1;
            __delay_us(1000);
            PORTAbits.RA0 = 0;
            __delay_ms(1000);
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("DISPENSANDO");
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String(" COMIDA...");
            __delay_ms(5000);
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String(" RH:   T:   L%:");
            __delay_ms(500);
       // Lcd_Write_String("000");    
        }
        else if(LDR>80 && cont1==1){
            // HACEMOS LA APERTURA DLE SERVO PARA SERVIR COMIDA
            cont1 = 0;
            PORTAbits.RA0 = 1;
            __delay_us(1000);
            PORTAbits.RA0 = 0;
            __delay_ms(20);
            __delay_ms(1000);
            // UNICAMENTE SE MUESTRA QUE ESTAMOS DISPENSANDO COMIDA
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String("DISPENSANDO");
            Lcd_Set_Cursor(2,1);
            Lcd_Write_String(" COMIDA...");
            __delay_ms(5000);
            
            // RESET AL LCD PARA VOLVER A LA INTERFAZ NORMAL
            Lcd_Clear();
            Lcd_Set_Cursor(1,1);
            Lcd_Write_String(" RH:   T:   L%:");
            __delay_ms(500);
    }
    }
    return;
}



void __interrupt()isr(void){
    if(PIR1bits.RCIF == 1){
        tempRX = RCREG;
        if(tempRX ==0x00){
           if (PIR1bits.TXIF == 1){
        switch(F1){
            case 0:
                TXREG = unit0;  // 48 MEANS ASCII FOR ZERO.
                F1 = 1;
                break;
            case 1:
                TXREG = dec0;
                F1 = 0;
                break;
        }
        PIR1bits.RCIF = 0;
        TXIF=0;
    } 
        }
    
}
}
void str_2_dc(uint16_t var){       // PARA PODER OBTENER EL VALOR EN DECIMAL
    uint16_t vcv;
    vcv = var;                  
    unit0_0 = (vcv/100) ;              //PRIMERO EN SIGNIFICANCIA
    vcv = (vcv - (unit0_0*100));
    dec0_12 = (vcv/10);              //SEGUNDO EN SIGNIFICANCIA
    vcv = (vcv - (dec0_12*10));
    dec1_12 = (vcv);                //TERCERO EN SIGNIFICANCIA
    unit0_0 = unit0_0 + 48;          //ASCII
    dec0_12 = dec0_12 + 48;
    dec1_12 = dec1_12 + 48;
    
}

// CONFIGURACION DEL MASTER


void initSETUP(void){
    TRISA = 0; // SERVO y BOMBA DE AGUA
    TRISB = 0;
    TRISC = 0b11000000;
    TRISD = 0;
    TRISE = 0;
    PORTE = 0;
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    ANSEL = 0;
    ANSELH = 0;
    OSCCONbits.IRCF2 = 1; // 8MHz 
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    BAUDCTLbits.BRG16 = 1;
    SPBRG = 207;
    SPBRGH = 0;
    RCSTAbits.SPEN = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1; 
    TXSTAbits.TXEN = 1; 
    // MAIN INTERRUPTIONS
    INTCONbits.GIE = 1;
    INTCONbits.PEIE =1;
    I2C_Master_Init(100000); //SET AL TIMER DEL MASTER
    return;
}