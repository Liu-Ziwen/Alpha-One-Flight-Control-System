/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    bsp_mpu6050.h
  */
  
#include "main.h"

BSP_MPU6K mpu6k;

//初始化MPU6K的片选信号引脚和中断线
void BSP_MPU6K::init_io(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	//PB14:MPU_CS
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIOB->BSRRL=GPIO_Pin_14;
	//PC5:MPU_INT
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//mpu6k配置寄存器
void BSP_MPU6K::set_reg(uint8_t addr, uint8_t data)
{
	MPU6K_ENABLE;
	hal_spi.set_reg(addr&0x7F,data);
	MPU6K_DISABLE;
}

////mpu6k读取寄存器
//uint8_t BSP_MPU6K::read_reg(uint8_t addr)
//{
//	uint8_t retval;
//	MPU6K_ENABLE;
//	retval = hal_spi.read_one_reg(addr|0x80);
//	MPU6K_DISABLE;
//	return retval;
//}

//mpu6k读取指定长度的字节
void BSP_MPU6K::read_len(uint8_t addr, uint8_t len, uint8_t *data)
{
	MPU6K_ENABLE;
	hal_spi.read_multi_reg(addr|0x80, len, data);
	MPU6K_DISABLE;
}

//mpu6k初始化
//包括IO初始化变量have_imu初始化
void BSP_MPU6K::init(void)
{
	init_io();
	delay.ms(100);
	set_reg(PWR_MGMT_1, DEVICE_RESET);
	delay.ms(100);
	set_reg(SGN_PATH_RST,0x07);			//传感器复位（见数据手册说明）
	delay.ms(100);
	set_reg(PWR_MGMT_1,CLKSEL_CODE3);	//设置时钟
	delay.ms(5);

	set_reg(PWR_MGMT_2,PWR_MGMT_2_RESET);	//禁止休眠
	delay.ms(1);
	set_reg(USER_CTRL,I2C_IF_DIS);		//失能I2C，使能SPI
	delay.ms(1);
	set_reg(CONFIG,DLPF_CFG_4);			//21Hz低通滤波(Accelerometer:21Hz;Gyroscope:20Hz;)
	delay.ms(1);
	set_reg(SMPLRT_DIV,SMPLRT_1KHZ);	//采样率1KHz
	delay.ms(1);
	set_reg(GYRO_CONFIG,GYRO_200DPS);	//GYRO量程：±2000°/s
	delay.ms(1);
	set_reg(ACCEL_CONFIG,ACCEL_8G);		//ACC量程：±8g
	delay.ms(1);
	set_reg(INT_ENABLE,RAW_RDY_EN);		//中断线使能
	delay.ms(1);
	set_reg(INT_PIN_CFG,INT_ANYRD_2CLEAR|LATCH_INT_EN);	//中断线配置：任何读操作会清零中断并拉低中断引脚
	delay.ms(1);
	
	have_imu = false;
	
}

//检查ID号
bool BSP_MPU6K::check_WHOAMI(void)
{
	uint8_t value;
	read_len(WHO_AM_I,1,&value);
	if(value == WHO_AM_I_VALUE)
		return true;
	else return false;
}

//带有检查ID号的初始化
void BSP_MPU6K::init_with_check(void)
{
	static uint8_t err_count = 0;
	do
	{
		init();
		have_imu = check_WHOAMI();
		err_count++;
		if(err_count>=5)break;
	}while(!have_imu);
}

//更新传感器数据
void BSP_MPU6K::update(void)
{
	uint8_t buf[14];
	if(!have_imu)return;
	
	read_len(ACCEL_XOUT_H, 14, buf);
	
	raw_data.ACC_X = COM_Joint_Two8Bit_To_16Bit(buf[0], buf[1]);
	raw_data.ACC_Y = -COM_Joint_Two8Bit_To_16Bit(buf[2],buf[3]);	
	raw_data.ACC_Z = -COM_Joint_Two8Bit_To_16Bit(buf[4],buf[5]);
	raw_data.IMU_TEMP =COM_Joint_Two8Bit_To_16Bit(buf[6],buf[7]);
	raw_data.GYR_X = COM_Joint_Two8Bit_To_16Bit(buf[8],buf[9]);
	raw_data.GYR_Y = -COM_Joint_Two8Bit_To_16Bit(buf[10],buf[11]);
	raw_data.GYR_Z = -COM_Joint_Two8Bit_To_16Bit(buf[12],buf[13]);
	
	raw_data.ACC_X = raw_data.ACC_X - raw_offset.ACC_X ;
	raw_data.ACC_Y = raw_data.ACC_Y - raw_offset.ACC_Y ;
	raw_data.ACC_Z = raw_data.ACC_Z - raw_offset.ACC_Z ;
	raw_data.GYR_X = raw_data.GYR_X - raw_offset.GYR_X ;
	raw_data.GYR_Y = raw_data.GYR_Y - raw_offset.GYR_Y ;
	raw_data.GYR_Z = raw_data.GYR_Z - raw_offset.GYR_Z ;
	
	IMU_SensorData.ACC_X = raw_data.ACC_X * MPU6K_ACCEL_SCALE;
	IMU_SensorData.ACC_Y = raw_data.ACC_Y * MPU6K_ACCEL_SCALE;
	IMU_SensorData.ACC_Z = raw_data.ACC_Z * MPU6K_ACCEL_SCALE;
	IMU_SensorData.IMU_TEMP = raw_data.IMU_TEMP / 340.0f + 36.53f;
	IMU_SensorData.GYR_X = raw_data.GYR_X * MPU6K_GYRO_SCALE;
	IMU_SensorData.GYR_Y = raw_data.GYR_Y * MPU6K_GYRO_SCALE;
	IMU_SensorData.GYR_Z = raw_data.GYR_Z * MPU6K_GYRO_SCALE;
}	




