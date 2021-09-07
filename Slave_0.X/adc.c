/*
 * File:   adc.c
 * Author: Fernando Sandoval
 *
 * Created on July 21, 2021, 6:44 PM
 */


#include <xc.h>
#include <stdint.h>
#include "adc.h"
#define _XTAL_FREQ 4000000


void ADC_START() {
    if (ADCON0bits.GO == 0){
        ADCON0bits.CHS = 0; //UTILIZAMOS EL CANAL 0 
        __delay_us(200);
        ADCON0bits.GO = 1;
    }
    return;
}
