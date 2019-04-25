/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    hal_dma.h
  */ 

#ifndef __HAL_DMA_H_
#define __HAL_DMA_H_

//��������кܶ��ļ����õ������COMx�����Ƿŵ�define.hͷ�ļ���
#define COM1	1
#define COM2	2
#define COM3	3
#define COM4	4
#define COM5	5
#define COM6	6

#define COM4_TX_BUFFER_SIZE 	100	//����4�������ݳ���(�����͵���վ)
#define COM4_RX_BUFFER_SIZE 	50	//����4�������ݳ���

#define COM5_TX_BUFFER_SIZE 	50	//����5�������ݳ���(GPS)
#define COM5_RX_BUFFER_SIZE 	1	//����5�������ݳ��� 
///////////////////////////////////////////
#ifdef __cplusplus

class HAL_DMA
{
public:
	void init(void);
	void transfer(uint8_t COMx, uint16_t buffer_size);
	void clear_tx_flag(uint8_t COMx);
private:
	void tx_config(DMA_Stream_TypeDef *DMAx_Streamx, uint32_t DMA_Channel_x, uint32_t peripheral_addr, uint32_t memory_addr, uint16_t buffer_size);
	void nvic_config(uint8_t COMx);
	void enable(DMA_Stream_TypeDef *DMAx_Streamx, uint16_t buffer_size);
};

extern HAL_DMA hal_dma;

#endif

extern bool com4_tx_over;
extern bool com5_tx_over;

extern uint8_t com4_tx_buf[COM4_TX_BUFFER_SIZE];
extern uint8_t com4_rx_buf[COM4_RX_BUFFER_SIZE];

extern uint8_t com5_tx_buf[COM5_TX_BUFFER_SIZE];
extern uint8_t com5_rx_buf[COM5_RX_BUFFER_SIZE];

#endif

