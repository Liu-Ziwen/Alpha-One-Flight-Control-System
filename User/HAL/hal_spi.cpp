/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_spi.cpp
  */

#include "main.h"

HAL_SPI hal_spi;
//SPI_BaudRatePrescaler_4;

void HAL_SPI::init(uint16_t baud)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;
	//ʱ��ʹ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	//���Ÿ���
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //SPI2_SCK
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1); //SPI2_MISO
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1); //SPI2_MOSI
	//GPIO����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//SPI����
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//���ݷ���2��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//����ģʽ
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//���ݳ��ȣ�8λ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//ʱ�������ز�������
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//ʱ�ӵĵ�1�����ز�������
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//Ƭѡ���Ʒ�ʽ���������
	SPI_InitStructure.SPI_BaudRatePrescaler = baud;						//MS5611����٣�20MHz���ɲ���8��Ƶ(10.5MHz)
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//����λ������򣻸�λ�ȴ�
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC����ʽ�Ĵ�������λ��Ϊ7�������̲���
	SPI_Init(SPI1, &SPI_InitStructure);
		
	SPI_Cmd(SPI1,DISABLE);
	SPI_Cmd(SPI1, ENABLE);
	rw_byte(0xFF);	//�������䣬ά��MOSIΪ�ߵ�ƽ
}

//SPI2��дһ���ֽ�
uint8_t HAL_SPI::rw_byte(uint8_t txdata)
{
	uint16_t retry = 0;
	
	//�ȴ����ͻ�����Ϊ�գ���ʱ�˳�������һ��byte
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>=0xFFFF)return 0;
	}
	SPI_I2S_SendData(SPI1, txdata);
	retry = 0;
	
	//�ȴ����ջ�����Ϊ�գ���ʱ�˳������ؽ��յ�������
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;
		if(retry>=0xFFFF)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1);
}

//����Χ������ĳ����ַд����Ӧ������
void HAL_SPI::set_reg(uint8_t addr,uint8_t data)
{
	rw_byte(addr);
	rw_byte(data);
}

//����Χ������ĳ���Ĵ�����ȡ��Ӧ������
uint8_t HAL_SPI::read_one_reg(uint8_t addr)
{
	rw_byte(addr);
	return rw_byte(0x00);
}

//����Χ������ĳ���Ĵ�����ȡ�����Ӧ�ļĴ���
void HAL_SPI::read_multi_reg(uint8_t addr,uint8_t len,uint8_t *data)
{
	uint8_t i = 0;
	rw_byte(addr);
	for(i = 0;i < len;i++)
		data[i] = rw_byte(0);
}





