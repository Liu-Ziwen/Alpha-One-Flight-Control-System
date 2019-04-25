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
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	//引脚复用
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1); //SPI2_SCK
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1); //SPI2_MISO
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1); //SPI2_MOSI
	//GPIO配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	//SPI配置
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//数据方向：2线全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						//主机模式
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					//数据长度：8位
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							//时钟上升沿采样数据
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						//时钟的第1个边沿采样数据
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							//片选控制方式：软件控制
	SPI_InitStructure.SPI_BaudRatePrescaler = baud;						//MS5611最高速：20MHz，可采用8分频(10.5MHz)
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					//数据位传输次序；高位先传
	SPI_InitStructure.SPI_CRCPolynomial = 7;							//CRC多项式寄存器，复位后为7。本工程不用
	SPI_Init(SPI1, &SPI_InitStructure);
		
	SPI_Cmd(SPI1,DISABLE);
	SPI_Cmd(SPI1, ENABLE);
	rw_byte(0xFF);	//启动传输，维持MOSI为高电平
}

//SPI2读写一个字节
uint8_t HAL_SPI::rw_byte(uint8_t txdata)
{
	uint16_t retry = 0;
	
	//等待发送缓冲区为空，超时退出，发送一个byte
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>=0xFFFF)return 0;
	}
	SPI_I2S_SendData(SPI1, txdata);
	retry = 0;
	
	//等待接收缓冲区为空，超时退出，返回接收到的数据
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
		retry++;
		if(retry>=0xFFFF)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1);
}

//对外围器件的某个地址写入相应的数据
void HAL_SPI::set_reg(uint8_t addr,uint8_t data)
{
	rw_byte(addr);
	rw_byte(data);
}

//对外围器件的某个寄存器读取相应的数据
uint8_t HAL_SPI::read_one_reg(uint8_t addr)
{
	rw_byte(addr);
	return rw_byte(0x00);
}

//对外围器件的某个寄存器读取多个相应的寄存器
void HAL_SPI::read_multi_reg(uint8_t addr,uint8_t len,uint8_t *data)
{
	uint8_t i = 0;
	rw_byte(addr);
	for(i = 0;i < len;i++)
		data[i] = rw_byte(0);
}





