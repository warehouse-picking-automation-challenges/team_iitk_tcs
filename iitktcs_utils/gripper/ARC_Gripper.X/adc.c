#include <xc.h>
#include "adc.h"
#include "config.h"
#include "Gpio.h"
void adc_init()
{
    /*ADCON0      = 0b00001010;
    ADCON1      = 0x00;
    ADCON2      = 0b10101111;
    ADCON3      = 0b11000000;
    ADCHS       = 0b11110011;*/

    ADCON0      = 0b00001110;
    ADCON1      = 0x00;
    ADCON2      = 0b10101111;
    ADCON3      = 0b11000000;
    ADCHS       = 0b00111111;
    
    ADCON0bits.ADON     = 0;  
}

unsigned int adc_read(unsigned short channel_id)
{
    switch (channel_id)
    {
        case M1_pot:
        {
            ADCON0      = 0b00000010;
            ADCHS       = 0b11111100;  
            break;
        }
        case M2_pot:
        {
            ADCON0      = 0b00000110;
            ADCHS       = 0b11001111;  
            break;
        }
        case FSR_1:
        {
            ADCON0      = 0b00001010;
            ADCHS       = 0b11110011;  
            break;
        }
        case FSR_2:
        {
            ADCON0      = 0b00001110;
            ADCHS       = 0b00111111;  
            break;
        }
        default:
        {
            return 0;
        }
    }

    ADCON0bits.ADON     = 1;
    ADCON0bits.GO   =   1;
    __delay_ms(1);// need to set right delay, refer datasheet , 1ms is not correct
    while(GO_nDONE);
    ADCON0bits.ADON     = 0;
    return ((ADRESH<<8)+ADRESL);
}