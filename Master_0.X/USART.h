/*
 * File:   USART.h
 * Author: Fernando Sandoval
 *
 * Created on September 8, 2021, 1:53 AM
 */

#ifndef __USART_H
#define	__USART_H

#include <xc.h> 
#include <pic16f887.h>
#include <stdint.h>

void init_USART (void);
char USART_Receive(void);
void USART_Transmit(char dato);

#endif