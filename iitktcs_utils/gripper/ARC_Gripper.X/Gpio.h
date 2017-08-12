
#ifndef GPIO_H
#define	GPIO_H


#define STA_LED1    PORTCbits.RC0
#define STA_LED2    PORTCbits.RC1
#define PIN_OUTPUT  0
#define PIN_INPUT   1

#define M1_pot  0
#define M2_pot  1
#define FSR_1   2
#define FSR_2   3

#define M1_EN   PORTDbits.RD0
#define M2_EN   PORTDbits.RD1
#define PWM0    PORTBbits.RB0
#define PWM2    PORTBbits.RB2
#define PWM4    PORTDbits.RD5
#define PWM6    PORTDbits.RD6

void Gpio_Init(void);

#endif	/* GPIO_H */

