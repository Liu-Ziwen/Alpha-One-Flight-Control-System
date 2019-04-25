/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    08-April-2019
  * @file    hal_uart.h
  */ 

#ifndef __HAL_UART_H_
#define __HAL_UART_H_

#define UART_BUFFER_SIZE	512
#define TX_LEN				92
#define RX_LEN				79

#ifdef __cplusplus

class HAL_UART
{
public:
	void init(void);
	void com4_init(uint32_t baudrate);
	void com4_send_byte(uint8_t data);
	void com5_init(uint32_t baudrate);
};

extern HAL_UART hal_uart;


extern uint8_t test_temp[UART_BUFFER_SIZE];
extern uint8_t uart_interrupt_temp;
extern uint16_t uart_interrupt_temp_n;


#endif
#endif






