#include <xc.h>
#include <stdint.h> 
#include <stdio.h>


// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT        // Oscillator Selection bits (XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)


#define _XTAL_FREQ 8000000

void setup(void);
void PWMccp1out(unsigned int duty, unsigned int fpwm);


//************************Codigo principal************************
void main() {
OSCCON= 0b01110011;        //configuración del Oscilador
//ADCON1=0xff; //todo los puertos definidos como digitales
setup();
//TRISAbits.TRISA5=0;

//TRISD=0;
while(1){
    
    //PWMccp1out(100,50);
    //__delay_ms(5000);   
    /*PWMccp1out(0,50);
    __delay_ms(5000);*/
    
    
    
   /* if (PORTAbits.RA0==1){
        
       // PWMccp1out(100,50); 
        for(int i=0; i<100; i++)
        {
           // PORTAbits.RA5=1;
            PWMccp1out(i,50); 
            __delay_ms(10);
        }
        __delay_ms(1000);
    }*/
    if (PORTAbits.RA0 == 0){
                __delay_ms(50);
                if (PORTAbits.RA0 == 1){
                    for(int i=0; i<100; i++) {
           // PORTAbits.RA5=1;
            PWMccp1out(i,50); 
            __delay_ms(10);
                    }
        __delay_ms(5000);
        
        for(int i=100; i>0; i--)
        {
         //   PORTAbits.RA5=0;
            PWMccp1out(i,50); 
            __delay_ms(10);
        }
        __delay_ms(1000);
                }
            }
    
     /*if (PORTAbits.RA0==0)
     {
        // PWMccp1out(0,50);
         
        for(int i=100; i>0; i--)
        {
         //   PORTAbits.RA5=0;
            PWMccp1out(i,50); 
            __delay_ms(10);
        }
        __delay_ms(1000);
     }*/  


}

}
void setup (void){
    PORTA=0;
    TRISAbits.TRISA0=1;
    ANSEL=0;
    
    
}
void PWMccp1out(unsigned int duty, unsigned int fpwm){

    TRISCbits.TRISC2=0;
    unsigned int prescalador=1;
    unsigned long PR2cal=_XTAL_FREQ;
    PR2cal=PR2cal/fpwm;
    PR2cal=PR2cal/4;  
    T2CON=0b00000000;
    while(PR2cal>256){
       prescalador=prescalador*4;
       T2CON++;
       PR2cal=PR2cal/4;
    }
    PR2cal--;
    PR2=PR2cal; 
    unsigned long AuxCCPR1L=PR2cal;
    AuxCCPR1L++;
    AuxCCPR1L=AuxCCPR1L*duty;
    AuxCCPR1L=AuxCCPR1L*4;
    AuxCCPR1L=AuxCCPR1L/100;
    unsigned int AuxCCP1CON= AuxCCPR1L;
    AuxCCP1CON= AuxCCP1CON & 0b00000011 ;
    AuxCCP1CON= AuxCCP1CON*16;//<< 4
    CCP1CON= 0b00001100;
    CCP1CON= CCP1CON|AuxCCP1CON;
    AuxCCPR1L=AuxCCPR1L/4; //>>2
    CCPR1L=AuxCCPR1L;
    T2CONbits.TMR2ON=1;
    TMR2=0;       
}
