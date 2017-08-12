#ifndef EUSART_H
#define	EUSART_H

#define ASYNC_MODE 			0
#define SYNC_MASTER_MODE 	1
#define SYNC_SLAVE_MODE		2

#define TX_8_BIT	0
#define TX_9_BIT	1

#define RX_INT_ON   1
#define RX_INT_OFF  0

void eusart_init(unsigned char usart_mode, char tx_mode, unsigned long fosc, unsigned int baud, unsigned char interrupt_mode);
void uart_send_byte(unsigned char byte);
unsigned char uart_recv_byte(void);
void uart_send_pkt(unsigned char *pkt, unsigned char size);
unsigned char uart_recv_pkt(unsigned char *pkt, unsigned char size);


#endif	/* EUSART_H */

