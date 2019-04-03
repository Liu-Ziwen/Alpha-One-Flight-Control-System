/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    bsp_hmc5883l.cpp
  */

#include "main.h"

BSP_HMC5883L hmc5883l;

void BSP_HMC5883L::write_reg(uint8_t addr,uint8_t data)
{
	hal_i2c.write_multi_bytes(HMC5883L_WRITE_ADDR, addr, 1, &data);
}

uint8_t BSP_HMC5883L::read_reg(uint8_t addr)
{
	uint8_t retval=0;
	hal_i2c.read_multi_bytes(HMC5883L_READ_ADDR,addr,1,&retval);
	return retval;
}

//从给定寄存器起始地址读取给定长度的数据：
void BSP_HMC5883L::read_len(uint8_t addr,uint8_t len,uint8_t *data)
{
	hal_i2c.read_multi_bytes(HMC5883L_READ_ADDR,addr,len,data);
}


void BSP_HMC5883L::init(void)
{
	write_reg(HMC5883L_R_CONFA,0x78);	//采样平均数8 输出速率75Hz 正常测量
	write_reg(HMC5883L_R_CONFB,0x40);	//default ±1.9Ga  增益820LSb/Gauss  数字分辨率1.22mG/LSb
	write_reg(HMC5883L_R_MODE,0x00);	//default Single-Measurement Mode: 0x01\Continuous: 0x00（使用连续读取）
}

		
bool BSP_HMC5883L::check_WHOAMI(void)
{
	uint8_t buf[3];
	read_len(HMC5883L_R_IDA,3,buf);
	if((buf[0]==0x48)&&(buf[1]==0x34)&&(buf[2]==0x33))
		return true;
	else return false;
}



void BSP_HMC5883L::init_with_check(void)
{
	static uint8_t err_count = 0;
	//检查是否有磁罗盘
	do
	{
		init();
		have_compass = check_WHOAMI();
		err_count++;
		if(err_count>=5)break;
	}while(!have_compass);
}


void BSP_HMC5883L::update(void)
{
	uint8_t buf[6];
	
	if(!have_compass)return;

	read_len(HMC5883L_R_XMSB,6,buf);
	
	raw_data.MAG_X = COM_Joint_Two8Bit_To_16Bit(buf[0],buf[1]);
	raw_data.MAG_Z = COM_Joint_Two8Bit_To_16Bit(buf[2],buf[3]);
	raw_data.MAG_Y = COM_Joint_Two8Bit_To_16Bit(buf[4],buf[5]);
	
	Compass_SensorData.MAG_X = raw_data.MAG_X;
	Compass_SensorData.MAG_Y = raw_data.MAG_Y;
	Compass_SensorData.MAG_Z = raw_data.MAG_Z;
		
}








