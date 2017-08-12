#include "config.h"
#include <stdio.h>
#include <pic18f4431.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include "oscillator.h"
#include "Gpio.h"
#include "eusart.h"
#include "adc.h"
#include "common.h"

#define MOTOR_STUCK 1
#define MOTOR_N_STUCK 0

unsigned int n_sensor1, n_sensor2, n_mot1, n_mot2;
unsigned char uc_arr_buffer [40];
uint8_t u8_arr_buffer[8];
uint16_t u16_diff_angle, u16_current_angle_m1, u16_current_angle_m2, u16_target_angle_m1, u16_target_angle_m2;
unsigned char uc_arr_rcv_buff[4];
unsigned char uc_rx_flag;

void main()
{
	Osc_Init(Mhz_8);
    Gpio_Init();
   
    STA_LED1 = 0;
    STA_LED2 = 1;
    adc_init();
    eusart_init(ASYNC_MODE, TX_8_BIT, 8000000UL, 9600, RX_INT_ON );
    M1_EN   =   0;
    M2_EN   =   0;
    u8_arr_buffer[0] = 0x7e;
    u8_arr_buffer[7] = 0xe7;
    uint16_t error = 0, integral = 0, last_error = 0, derivative = 0;
    uint16_t err_m1 = 0 , lerr_m1 = 0, err_m2 = 0, lerr_m2 = 0;
    uint16_t m1_stuck_cnt = 0, m2_stuck_cnt = 0, m1_chng_ang = 0, m2_chng_ang = 0;
    uint8_t m1_Flag = 0, m2_Flag = 0;
  while(1)
  {
      if(uc_rx_flag == RX_DATA_RDY)
      {
         u16_target_angle_m1 = uc_arr_rcv_buff[1];
         u16_target_angle_m2 = uc_arr_rcv_buff[2];
         uc_rx_flag = RX_DATA_CLR;
         m1_Flag = MOTOR_N_STUCK;
         m2_Flag = MOTOR_N_STUCK;
      }
        STA_LED1 = ~ STA_LED1;
        STA_LED2 = ~ STA_LED2;
        n_sensor1 = adc_read(FSR_1);
        n_sensor2 = adc_read(FSR_2);
        n_mot1    = adc_read(M1_pot);
        n_mot2    = adc_read(M2_pot);

        u8_arr_buffer[1] = (n_sensor1 & 0xff);
        u8_arr_buffer[2] = ((n_sensor1 >> 8 ) & 0xff);
        u8_arr_buffer[3] = (n_sensor2 & 0xff);
        u8_arr_buffer[4] = ((n_sensor2 >> 8 ) & 0xff);

        u16_current_angle_m1 = (uint16_t)(n_mot1/10);
        u16_current_angle_m2 = (uint16_t)(n_mot2/10);

        u8_arr_buffer[5] = (uint8_t)u16_current_angle_m1;
        u8_arr_buffer[6] = (uint8_t)u16_current_angle_m2;
/*----------------------------Motor 1 -------------------------------*/        
        err_m1 = abs(u16_current_angle_m1 - u16_target_angle_m1);
        if(err_m1 < 2)
        {
            u16_current_angle_m1 = u16_target_angle_m1;
            M1_EN   =   0;
            m1_stuck_cnt = 0;
        }
        m1_chng_ang = lerr_m1 - err_m1;
        
        if(m1_chng_ang == 0)
            m1_stuck_cnt ++;
        else
            m1_stuck_cnt = 0;
        
        lerr_m1 = err_m1;

        if(m1_stuck_cnt > 20)
        {
            m1_stuck_cnt = 0;
            m1_Flag = MOTOR_STUCK;
        }
        if(m1_Flag == MOTOR_N_STUCK)
        {
            if(u16_current_angle_m1 < u16_target_angle_m1)
            {
                M1_EN   =   1;
                PWM0    =   1;
                PWM2    =   0;
            }
            else if(u16_current_angle_m1 > u16_target_angle_m1)
            {
                M1_EN   =   1;
                PWM0    =   0;
                PWM2    =   1;
            }
        }        
/*----------------------------Motor 2 -------------------------------*/        
        err_m2 = abs(u16_current_angle_m2 - u16_target_angle_m2);        
        if(err_m2 < 2)
        {
            u16_current_angle_m2 = u16_target_angle_m2;
            M2_EN   =   0;
            m2_stuck_cnt = 0;
        }
        m2_chng_ang = lerr_m2 - err_m2;
        
        if(m2_chng_ang == 0)
            m2_stuck_cnt ++;
        else
            m2_stuck_cnt = 0;
        
        lerr_m2 = err_m2;

        if(m2_stuck_cnt > 20)
        {
            m2_stuck_cnt = 0;
            m2_Flag = MOTOR_STUCK;
        }
        if(m2_Flag == MOTOR_N_STUCK)
        {
            if(u16_current_angle_m2 < u16_target_angle_m2)
            {
                M2_EN   =   1;
                PWM4    =   1;
                PWM6    =   0;
            }
            else if(u16_current_angle_m2 > u16_target_angle_m2)
            {
                M2_EN   =   1;
                PWM4    =   0;
                PWM6    =   1;
            }
        }
        //sprintf(uc_arr_buffer,"%d  %d  %d  %d  %d\r\n",n_sensor1,n_sensor2,u16_current_angle_m1,u16_current_angle_m2,u16_diff_angle);
        //sprintf(uc_arr_buffer,"CA:%d error:%d cn_ang:%d Cnt:%d stk:%d \r\n",u16_current_angle_m1,err_m1,m1_chng_ang,m1_stuck_cnt,m1_Flag);
        //uart_send_pkt(uc_arr_buffer,40);
        uart_send_pkt(u8_arr_buffer,8);
    __delay_ms(10);
    
  }
}

