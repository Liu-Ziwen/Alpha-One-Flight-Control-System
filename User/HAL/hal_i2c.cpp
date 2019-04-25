/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_i2c.cpp
  */

#include "main.h"

HAL_I2C hal_i2c;

void HAL_I2C::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef	I2C_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_I2C2);
		
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
      
	I2C_Cmd(I2C2, DISABLE);
	I2C_DeInit(I2C2);	
	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
	I2C_InitStructure.I2C_Ack=I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed=400000; 
	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
	I2C_Init(I2C2, &I2C_InitStructure);   
	
	I2C_Cmd(I2C2, ENABLE);
	
}


bool HAL_I2C::wait_for_event(uint32_t event)
{
	uint32_t tries = 0;

	while(!I2C_CheckEvent(I2C2, event))
	{
		tries++;
		if(tries > I2C_ACK_TRIES)
		{
			err_count++;
			init();
			return false;
		}
	}	
	return true;
}


bool HAL_I2C::check_bus_busy(void)
{
	uint32_t tries = 0;

	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY))
	{
		tries++;
		if(tries > I2C_BUSY_TRIES)
		{
			err_count++;
			init();
			return true;
		}
	}	
	return false;
}


bool HAL_I2C::write_multi_bytes(uint8_t slave_addr, uint8_t write_addr, uint8_t len, uint8_t *data)
{
	I2C_GenerateSTART(I2C2, ENABLE);                                        //产生起始位		
	if(!wait_for_event(I2C_EVENT_MASTER_MODE_SELECT))
	return false;
	I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);       //发送器件地址
	if(!wait_for_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	return false;
	I2C_SendData(I2C2, write_addr);                                         //寄存器具体地址
	if(!wait_for_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	return false;		
	while(len--)                                                          	//利用 while 循环 发送数据
	{
		I2C_SendData(I2C2, *data);                                          //发送数据
		data++;                                                             //数据指针移位
		if(!wait_for_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		return false;		
	}
	I2C_GenerateSTOP(I2C2, ENABLE);    	//产生停止信号
	return true;
}


bool HAL_I2C::read_multi_bytes(uint8_t slave_addr, uint8_t read_addr, uint8_t len, uint8_t *data)
{
	uint32_t tries = 0;
	if(check_bus_busy())
	return false;
	I2C_GenerateSTART(I2C2, ENABLE);//发送起始条件
	if(!wait_for_event(I2C_EVENT_MASTER_MODE_SELECT))
	return false;		
	I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Transmitter);//发送从机地址
	if(!wait_for_event(I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
	return false;		
	I2C_Cmd(I2C2, ENABLE);//通过设置PE位清除EV6
	I2C_SendData(I2C2, read_addr);//发送需要读的地址
	if(!wait_for_event(I2C_EVENT_MASTER_BYTE_TRANSMITTED))
	return false;		
	I2C_GenerateSTART(I2C2, ENABLE);//再次发送起始条件
	if(!wait_for_event(I2C_EVENT_MASTER_MODE_SELECT))
	return false;			
	I2C_Send7bitAddress(I2C2, slave_addr, I2C_Direction_Receiver);//发送从机地址	
	if(!wait_for_event(I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
	return false;			
	while(len && (tries < I2C_ACK_TRIES_MAX))
	{
		if(len == 1)
		{
			I2C_AcknowledgeConfig(I2C2, DISABLE);//禁止ACK
			I2C_GenerateSTOP(I2C2, ENABLE);//发送停止位
		}
		if(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED)) //查询EV7,并清除
		{
			*data = I2C_ReceiveData(I2C2);//读取一个字节
			data++;//指针下移
			len--;//待读取字节数递减
		}
		tries++;
	}	
	if(tries >= I2C_ACK_TRIES_MAX)
	{
		init();
		err_count++;
		return false;
	}	
	I2C_AcknowledgeConfig(I2C2, ENABLE);//使能ACK，为了再次读写	
	return true;
}


