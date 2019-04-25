/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    hal_dma.cpp
  */ 

#include "main.h"

/* COM����DMA�շ�������--------------------------------------------------------------------------------------------------- */
uint8_t com4_tx_buf[COM4_TX_BUFFER_SIZE];
uint8_t com4_rx_buf[COM4_RX_BUFFER_SIZE];

uint8_t com5_tx_buf[COM5_TX_BUFFER_SIZE];
uint8_t com5_rx_buf[COM5_RX_BUFFER_SIZE];
/////////////////////////////////////////////////
bool com4_tx_over = false;
bool com5_tx_over = false;

HAL_DMA hal_dma;

void HAL_DMA::init(void)
{
	//com4��ʼ��
	tx_config(DMA1_Stream4, DMA_Channel_4, (uint32_t)&UART4->DR, (uint32_t)com4_tx_buf, COM4_TX_BUFFER_SIZE);
	nvic_config(COM4);
	DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);//�򿪷�������ж�
	
	//com5��ʼ��
	tx_config(DMA1_Stream7, DMA_Channel_4, (uint32_t)&UART5->DR, (uint32_t)com5_tx_buf, COM5_TX_BUFFER_SIZE);
	nvic_config(COM5);
	DMA_ITConfig(DMA1_Stream7, DMA_IT_TC, ENABLE);//�򿪷�������ж�
}
void HAL_DMA::tx_config(DMA_Stream_TypeDef *DMAx_Streamx, uint32_t DMA_Channel_x, uint32_t peripheral_addr, uint32_t memory_addr, uint16_t buffer_size)
{
	DMA_InitTypeDef  DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ��

	DMA_DeInit(DMAx_Streamx);
	while (DMA_GetCmdStatus(DMAx_Streamx) != DISABLE);//�ȴ�DMA������
	
	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel = DMA_Channel_x;  						//ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = peripheral_addr;				//DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = memory_addr;					//DMA �洢��0��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;					//�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize = buffer_size;							//���ݴ����� 
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//���������ģʽ
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//�洢������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//�������ݳ���:8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							// ʹ����ͨģʽ 
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;					//�е����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;				//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;		//����ͻ�����δ���
	DMA_Init(DMAx_Streamx, &DMA_InitStructure);								//��ʼ��DMA Stream
}


//////
void HAL_DMA::nvic_config(uint8_t COMx)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	switch(COMx)
	{
		case COM4:
		{
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn; 
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
			NVIC_Init(&NVIC_InitStructure);
		}break;
		
		case COM5:
		{
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn; 
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; 
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
			NVIC_Init(&NVIC_InitStructure);
		}break;
	}
}


void HAL_DMA::enable(DMA_Stream_TypeDef *DMAx_Streamx, uint16_t buffer_size)
{
	DMA_Cmd(DMAx_Streamx, DISABLE);							//�ر�DMA����
	while(DMA_GetCmdStatus(DMAx_Streamx)!=DISABLE);			//ȷ��DMA���Ա�����
	DMA_SetCurrDataCounter(DMAx_Streamx, buffer_size);		//���ݴ�����
	DMA_Cmd(DMAx_Streamx, ENABLE);							//����DMA����
}


void HAL_DMA::transfer(uint8_t COMx, uint16_t buffer_size)
{
	switch(COMx)
	{
		case COM4:
		{
			USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
			enable(DMA1_Stream4,buffer_size);
			com4_tx_over=false;
		}break;
		
		case COM5:
		{
			USART_DMACmd(UART5,USART_DMAReq_Tx,ENABLE);
			enable(DMA1_Stream7,buffer_size);
			com5_tx_over=false;
		}break;
	}
}



void HAL_DMA::clear_tx_flag(uint8_t COMx)
{
	switch(COMx)
	{
		case COM4:
		{
			if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET) 
			{ 
				DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4); 
				com4_tx_over=true;
			}
		}break;
		
		case COM5:
		{
			if(DMA_GetFlagStatus(DMA1_Stream7,DMA_FLAG_TCIF7)!=RESET) 
			{ 
				DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7); 
				com5_tx_over=true;
			}
		}break;
	}	
}



extern "C"
{
	//COM4����
	void DMA1_Stream4_IRQHandler(void)
	{
		hal_dma.clear_tx_flag(COM4);
	}
	
	//COM5����
	void DMA1_Stream7_IRQHandler(void)
	{
		hal_dma.clear_tx_flag(COM5);
	}
}










