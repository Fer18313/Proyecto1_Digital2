/*
 * File:   USART.c
 * Author: Fernando Sandoval
 *
 * Created on September 8, 2021, 1:53 AM
 */
#include "USART.h"
#define _XTAL_FREQ 8000000
#include <xc.h>

void init_USART (void){
    TXSTAbits.SYNC = 0;  
    TXSTAbits.BRGH = 1;    
    BAUDCTLbits.BRG16 = 1;      
    SPBRG = 207;    
    SPBRGH = 0;    
    RCSTAbits.SPEN = 1; 
    RCSTAbits.RX9 = 0;  
    RCSTAbits.CREN = 1; 
    TXSTAbits.TXEN = 1; 
}
char USART_Receive(){                
    return RCREG; 
   }

void USART_Transmit(char dato){       
    while(TXSTAbits.TRMT == 0);
    TXREG = dato;
}


