#include <xc.h>
#include "oscillator.h"

void Osc_Init(unsigned char Osc_Freq)
{
	switch(Osc_Freq)
	{
		case Mhz_8:
		{
            OSCCON 	= 0b01110010; // Internal Oscillator with 8Mhz             
			break;
		}
		case Mhz_4:
		{
            OSCCON 	= 0b01100010; // Internal Oscillator with 8Mhz 
			break;
		}
		case Mhz_2:
		{
            OSCCON 	= 0b01010010; // Internal Oscillator with 8Mhz 
			break;
		}
        case Mhz_1:
		{
            OSCCON 	= 0b01000010; // Internal Oscillator with 8Mhz 
			break;
		}
        case Khz_500:
		{
            OSCCON 	= 0b00110010; // Internal Oscillator with 8Mhz 
			break;
		}
        case Khz_250:
		{
            OSCCON 	= 0b00100010; // Internal Oscillator with 8Mhz 
			break;
		}
        case Khz_125:
		{
            OSCCON 	= 0b00010010; // Internal Oscillator with 8Mhz 
			break;
		}
        case Khz_31:
		{
            OSCCON 	= 0b00000010; // Internal Oscillator with 8Mhz 
			break;
		}
                
	}
    
}
