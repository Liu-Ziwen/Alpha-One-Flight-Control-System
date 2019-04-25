/**
  ******************************************************************************
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @brief   hal_uart.cpp
  ******************************************************************************
**/	

#include "main.h"

uint8_t test_temp[UART_BUFFER_SIZE];
uint8_t uart_interrupt_temp;
uint16_t uart_interrupt_temp_n;
HAL_UART hal_uart;

void HAL_UART::init(void)
{
	com4_init(115200);
	com5_init(9600);
	hal_dma.init();
}


void HAL_UART::com4_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); 
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(UART4, &USART_InitStructure); 
  
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);   
	USART_Cmd(UART4, ENABLE); 
	USART_ClearFlag(UART4, USART_FLAG_TC);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
}

void HAL_UART::com4_send_byte(uint8_t data)
{
	while(USART_GetFlagStatus(UART4, USART_FLAG_TXE)==RESET);
	USART_SendData(UART4, data);
}

void HAL_UART::com5_init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); 

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_Init(GPIOC,&GPIO_InitStructure); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	USART_Init(UART5, &USART_InitStructure); 
  
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);   
	USART_Cmd(UART5, ENABLE); 
	USART_ClearFlag(UART5, USART_FLAG_TC);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

extern "C"
{
//	//
//	bool test_data_process(uint8_t rcvdata, uint8_t *data)
//	{
//		uint8_t k;
//		static uint8_t rcvPart = 0;
//		static uint8_t rcvDataNo = 0;
//		static uint8_t rcvDataLen = 0;  
//		static uint8_t rcvingBuf[10];

//		switch(rcvPart)
//		{
//				case 0:
//				if(rcvdata == 0x55)
//				{
//					rcvPart = 1;
//					rcvingBuf[rcvDataNo] = 0x55;
//					rcvDataNo++;
//				}
//				break;
//				
//				case 1:
//					if(rcvdata == 0xAA)
//					{
//						rcvPart =2;
//						rcvingBuf[rcvDataNo] = 0xAA;
//						rcvDataNo++;
//					}
//					else
//						rcvPart = 0;
//					break;

//				case 2:
//					rcvDataLen = 6;
//					rcvPart = 3;
//					rcvingBuf[rcvDataNo] = rcvdata;
//					rcvDataNo++;
//					break;
//				
//				case 3:
//					rcvingBuf[rcvDataNo] = rcvdata;
//					rcvDataNo++;
//					rcvDataLen--;
//					if(rcvDataLen == 0)
//					{
//						rcvPart = 4;
//					}
//					break;

//				case 4:
//					for(k = 0 ;k < 8 ; k++)
//					{
//						data[k] = rcvingBuf[k];
//					}
//					rcvPart = 0;
//					rcvDataNo = 0;
//					return true;

//				default:
//					break;
//			}
//		return false;
//	}
	
	//UART4中断服务程序
	void UART4_IRQHandler(void)
	{
//		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)
//		{
//			uart_interrupt_temp = USART_ReceiveData(UART4);
//			test_temp[uart_interrupt_temp_n++] = uart_interrupt_temp;

//			optical_flow_module_data_protocol(uart_interrupt_temp);
//			
//			USART_ClearITPendingBit(UART4,USART_IT_RXNE);
//			if(uart_interrupt_temp_n>=UART_BUFFER_SIZE)uart_interrupt_temp_n=0;
//		}
	}
}
