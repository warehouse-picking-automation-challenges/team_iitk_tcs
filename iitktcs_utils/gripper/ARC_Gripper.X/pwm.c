#include <xc.h>
#include "config.h"

void pwm_module_init(void)
{
    PWMCON0 = 0b01011111;
    PTCON0  = 0x00;
}
