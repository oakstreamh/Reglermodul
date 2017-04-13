/*
 * uart_buffer.h
 *
 * Created: 4/7/2017 1:18:08 PM
 *  Author: hjaek237
 */ 


#ifndef UART_BUFFER_H_
#define UART_BUFFER_H_

//Buffer size. To change buffer size, change here. It is now set to 64
#define BUFFER_SIZE 64

unsigned char uart0_rx_not_empty_flag;
unsigned char uart0_tx_not_empty_flag;
unsigned char uart0_rx_overf_flag;
unsigned char uart0_tx_overf_flag;
unsigned char uart0_rx_full_flag;
unsigned char uart0_tx_full_flag;


//uart0_send_byte is a function to place a byte in Tx_buffer
void uart0_send_byte(unsigned char value);

//uart0_get_byte is a function to retrieve a byte from Rx_buffer, which is returned
unsigned char uart0_get_byte(void);




#endif /* UART_BUFFER_H_ */