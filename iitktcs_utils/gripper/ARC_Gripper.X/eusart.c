#include <xc.h>
#include "eusart.h"
#include "common.h"

extern unsigned char uc_arr_rcv_buff[];
extern unsigned char uc_rx_flag;
unsigned short us_nHeader_detect;
unsigned short us_nNoOfRxdBytes = 1;
void eusart_init(unsigned char usart_mode, char tx_mode, unsigned long fosc, unsigned int baud, unsigned char interrupt_mode)
{
//	TRISC6 = 0;		// UART Tx enable
//	TRISC7 = 1;		// UART Rx enable
	switch(usart_mode)
	{
		case ASYNC_MODE:
		{
			BAUDCONbits.BRG16 = 0;
			SPBRG = (int)(fosc/(16UL * baud) -1);	// Formula varies based on UART settings
			TXSTA = 0b00100100;	// 8-Bit mode, Enable TX
			RCSTA = 0x90;
            if(interrupt_mode == RX_INT_ON)
            {
                RCIF = 0; //reset RX pin flag
                RCIP = 0; //Not high priority
                RCIE = 1; //Enable RX interrupt
                GIE  = 1; //Enable global interrupts
                PEIE = 1; //Enable peripheral interrupt (serial port is a peripheral)
            }
			break;
		}
		case SYNC_MASTER_MODE:
		{
			break;
		}
		case SYNC_SLAVE_MODE:
		{
			break;
		}		
	}
}


void uart_send_byte(unsigned char byte)
{
	while(!TXIF)
	{
		continue;
	}
	TXREG = byte;
}

unsigned char uart_recv_byte(void)
{
	while(!RCIF)
	{
		continue;
	}
	return RCREG;
}

void uart_send_pkt(unsigned char *pkt, unsigned char size)
{
	int i;
	for(i = 0; i < size; i++)
	{
		uart_send_byte(pkt[i]);
	}
}

unsigned char uart_recv_pkt(unsigned char *pkt, unsigned char size)
{
	int i;
	for(i = 0; i < size; i++)
	{
		pkt[i] = uart_recv_byte();
	}
	return size;
}

void interrupt SerialRxPinInterrupt()
{
    if(PIR1bits.RC1IF == 1)
    {     
        unsigned char uc_data = RCREG;
        if(uc_data == 0x7E)
        {
            us_nHeader_detect = 1;
            uc_arr_rcv_buff[0] = uc_data;
            goto SKIP;
        }
        if(uc_arr_rcv_buff[0] == 0x7E)
        {
            uc_arr_rcv_buff[us_nNoOfRxdBytes] = uc_data;
            if(us_nNoOfRxdBytes < 3)
            {
                us_nNoOfRxdBytes++;
            }
            else
            {
                us_nNoOfRxdBytes = 1;
                uc_rx_flag = RX_DATA_RDY;
            }
        }
    }
    SKIP:
    PIR1bits.RC1IF = 0;
            
}


