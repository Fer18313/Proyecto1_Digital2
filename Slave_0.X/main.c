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
// variables declaration
char Temperature[] = "Temp = 00.0 C  ";
char Humidity[]    = "RH   = 00.0 %  ";
unsigned char T_Byte1, T_Byte2, RH_Byte1, RH_Byte2, CheckSum ;

// send start signal to the sensor
void Start_Signal(void) {
  TRISA = 0;     // configure PORTAbits.RA0 as output
  PORTAbits.RA0 = 0;         // clear PORTAbits.RA0 output (logic 0)

  __delay_ms(25);        // wait 25 ms
  PORTAbits.RA0 = 1;         // set PORTAbits.RA0 output (logic 1)

  __delay_us(30);        // wait 30 us
  TRISA = 0b00000001;     // configure PORTAbits.RA0 as input
}
 
// Check sensor response
__bit Check_Response() {
  TMR1H = 0;                 // reset Timer1
  TMR1L = 0;
  TMR1ON = 1;                // enable Timer1 module

  while(!PORTAbits.RA0 && TMR1L < 100);  // wait until PORTAbits.RA0 becomes high (checking of 80µs low time response)

  if(TMR1L > 99)                     // if response time > 99µS  ==> Response error
    return 0;                        // return 0 (Device has a problem with response)

  else
  {
    TMR1H = 0;               // reset Timer1
    TMR1L = 0;

    while(PORTAbits.RA0 && TMR1L < 100); // wait until PORTAbits.RA0 becomes low (checking of 80µs high time response)

    if(TMR1L > 99)                   // if response time > 99µS  ==> Response error
      return 0;                      // return 0 (Device has a problem with response)

    else
      return 1;                      // return 1 (response OK)
  }
}

// Data read function
__bit Read_Data(unsigned char* dht_data)
{
  *dht_data = 0;

  for(char i = 0; i < 8; i++)
  {
    TMR1H = 0;             // reset Timer1
    TMR1L = 0;

    while(!PORTAbits.RA0)      // wait until PORTAbits.RA0 becomes high
      if(TMR1L > 100) {    // if low time > 100  ==>  Time out error (Normally it takes 50µs)
        return 1;
      }

    TMR1H = 0;             // reset Timer1
    TMR1L = 0;

    while(PORTAbits.RA0)       // wait until PORTAbits.RA0 becomes low
      if(TMR1L > 100) {    // if high time > 100  ==>  Time out error (Normally it takes 26-28µs for 0 and 70µs for 1)
        return 1;          // return 1 (timeout error)
      }

     if(TMR1L > 50)                  // if high time > 50  ==>  Sensor sent 1
       *dht_data |= (1 << (7 - i));  // set bit (7 - i)
  }

  return 0;                          // return 0 (data read OK)
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
    OSCCONbits.IRCF2 = 1; // 8MHZ
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1;
    OSCCONbits.SCS = 1;
    // MAIN INTERRUPTIONS
    INTCONbits.GIE = 1;
    INTCONbits.PEIE =1;
    INTCONbits.T0IE = 1;
    INTCONbits.T0IF =0;
    PIE1bits.TMR1IE = 1;
    PIR1bits.TMR1IF = 0;
    // PORTB INTERRUPT
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    TMR1H  = 0;           // reset Timer1
    TMR1L  = 0;
    IOCB = 0b01100000; // VALOR DEL PUERTO B
    OPTION_REGbits.nRBPU = 0; // PARA FACILITARNOS HAREMOS USO DE LA FUNCIONALIDAD DE PULL UP DEL PIC EN EL PUERTO B
    WPUB = 0b01100000; // VALOR DEL PUERTO B
    
    return;
}
void main(void)
{
  initSETUP();

  Lcd_Init();          // initialize LCD module
  __delay_ms(1000);
  while(1)
  {
    Start_Signal();     // send start signal to the sensor

    if(Check_Response())    // check if there is a response from sensor (If OK start reading humidity and temperature data)
    {
      // read (and save) data from the DHT11 sensor and check time out errors
      if(Read_Data(&RH_Byte1) || Read_Data(&RH_Byte2) || Read_Data(&T_Byte1) || Read_Data(&T_Byte2) || Read_Data(&CheckSum))
      {
        Lcd_Clear();       // clear LCD
        Lcd_Set_Cursor(2, 5);           // go to column 5, row 2
        Lcd_Write_String("Time out!");   // display "Time out!"
      }

      else         // if there is no time out error
      {
        if(CheckSum == ((RH_Byte1 + RH_Byte2 + T_Byte1 + T_Byte2) & 0xFF))
        {                                       // if there is no checksum error
          Temperature[7]  = T_Byte1 / 10  + '0';
          Temperature[8]  = T_Byte1 % 10  + '0';
          Temperature[10] = T_Byte2 / 10  + '0';
          Humidity[7]     = RH_Byte1 / 10 + '0';
          Humidity[8]     = RH_Byte1 % 10 + '0';
          Humidity[10]    = RH_Byte2 / 10 + '0';
          Temperature[11] = 223;    // put degree symbol (°)
          Lcd_Set_Cursor(1, 1);           // go to column 1, row 1
          Lcd_Write_Char(Temperature);
          Lcd_Set_Cursor(1, 2);           // go to column 1, row 2
          Lcd_Write_Char(Humidity);
        }

        // if there is a checksum error
        else
        {
          Lcd_Clear();       // clear LCD
          Lcd_Set_Cursor(1, 1);           // go to column 1, row 1
          Lcd_Write_String("Checksum Error!");
        }

      }
    }

    // if there is a response (from the sensor) problem
    else
    {
      Lcd_Clear();       // clear LCD
      Lcd_Set_Cursor(1, 4);           // go to column 3, row 1
      Lcd_Write_String("No response");
      Lcd_Set_Cursor(2, 1);           // go to column 1, row 2
      Lcd_Write_String("from the sensor");
    }

    TMR1ON = 0;        // disable Timer1 module
    __delay_ms(1000);  // wait 1 second

  }

}
// End of code.