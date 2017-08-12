#include <xc.h>
#include "Gpio.h"

void Gpio_Init(void)
{
    TRISCbits.RC0 = PIN_OUTPUT;
    PORTCbits.RC0 = PIN_OUTPUT;
    TRISCbits.RC1 = PIN_OUTPUT;
    PORTCbits.RC1 = PIN_OUTPUT;
    TRISCbits.RC6 = PIN_OUTPUT;
    PORTCbits.RC6 = PIN_OUTPUT;
    
    TRISBbits.RB0 = PIN_OUTPUT;
    TRISBbits.RB2 = PIN_OUTPUT;
    TRISDbits.RD0 = PIN_OUTPUT;
    TRISDbits.RD1 = PIN_OUTPUT;
    TRISDbits.RD5 = PIN_OUTPUT;
    TRISDbits.RD6 = PIN_OUTPUT;
    
    PORTBbits.RB0 = PIN_OUTPUT;
    PORTBbits.RB2 = PIN_OUTPUT;
    PORTDbits.RD0 = PIN_OUTPUT;
    PORTDbits.RD1 = PIN_OUTPUT;
    PORTDbits.RD5 = PIN_OUTPUT;
    PORTDbits.RD6 = PIN_OUTPUT;
    
    TRISAbits.TRISA2 = 1;   //Disable the output driver for pin RA2/AN2
    PORTAbits.RA2    = 1;
    TRISAbits.TRISA3 = 1;   //Disable the output driver for pin RA2/AN2
    PORTAbits.RA3    = 1;
}
