/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    bsp_ms5611.cpp
  */
  
#include "main.h"

BSP_MS5611 ms5611;

void BSP_MS5611::init_cs(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIO时钟
	//对CS片选信号的属性进行配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PB15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHZ
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化
	MS5611_DISABLE;//片选线拉高，不选中
}

//向MS5611发送一个指令
void BSP_MS5611::send_cmd(uint8_t data)
{
	MS5611_ENABLE;
	hal_spi.rw_byte(data);
	MS5611_DISABLE;
}

//从给定寄存器地址读取两个数据
uint16_t BSP_MS5611::read_two_bytes(uint8_t addr)
{
	uint8_t data[2];
	MS5611_ENABLE;
	hal_spi.read_multi_reg(addr,2,data);
	MS5611_DISABLE;
	return (((uint32_t)data[0])<<8) | ((uint32_t)data[1]);
}

//读取MS5611的出厂校准系数
void BSP_MS5611::get_calibration_coeff(void)
{
	uint16_t Calibration_Coeff[8];
	uint8_t i;
	for(i = 0; i < 8; i++)
		Calibration_Coeff[i] = read_two_bytes(CMD_MS5611_PROM_Setup + i*2);
	
	calibration_c1_SENS_T1 = Calibration_Coeff[1];
	calibration_c2_OFF_T1 = Calibration_Coeff[2];
	calibration_c3_TCS = Calibration_Coeff[3];
	calibration_c4_TCO = Calibration_Coeff[4];
	calibration_c5_T_REF = Calibration_Coeff[5];
	calibration_c6_TEMPSENS = Calibration_Coeff[6];
}

//读取MS5611的传感器数据，读取压力或者温度由指令决定
uint32_t BSP_MS5611::read_ADC(void)
{
	uint8_t data[3];
	MS5611_ENABLE;
	hal_spi.read_multi_reg(0,3,data);
	MS5611_DISABLE;
	return (((uint32_t)data[0])<<16) | (((uint32_t)data[1])<<8) | ((uint32_t)data[2]);
}

//对MS5611进行初始化
void BSP_MS5611::init(void)
{
	init_cs();
	send_cmd(CMD_MS5611_RESET);			//向MS5611发送复位指令
	delay.ms(100);
	get_calibration_coeff();			//读取出厂校准系数
	delay.ms(1);
}


////发送命令并读取未补偿的原始数据
void BSP_MS5611::read_uncompensated_data(void)
{
	if(stage == 0) 									//读取温度传感器数据
	{
		uncompensated_temp_D2 = read_ADC();			//读取一次温度数据
		stage++;
		send_cmd(CMD_CONVERT_D1_OSR4096);			//发送压力数据转换指令
	}
	else
	{
		uncompensated_pressure_D1 = read_ADC();		//读取一次压力数据
		data_updated = true;
		stage++;	
		if(stage == 5)
		{
			send_cmd(CMD_CONVERT_D2_OSR4096);		//第五次读取温度传感器值
			stage = 0;
		}
		else
		{
			send_cmd(CMD_CONVERT_D1_OSR4096);		//其他四次读取压力传感器值
		}
	}
}


void BSP_MS5611::calculate(void)
{
	int64_t dT,T2;
	int64_t TEMP;
	int64_t OFF,OFF2;
	int64_t SENS,SENS2;
	int32_t P;
	//计算公式参考MS5611数据手册的第8页
	dT = uncompensated_temp_D2 - ((int32_t)calibration_c5_T_REF  << 8);//实际和参考温度之间的差
	TEMP = 2000 + ((dT * calibration_c6_TEMPSENS) >> 23);//实际温度
	OFF = ((uint32_t)calibration_c2_OFF_T1 << 16) + ((calibration_c4_TCO * dT) >> 7);//实际温度的零偏
	SENS = ((uint32_t)calibration_c1_SENS_T1 << 15) +((calibration_c3_TCS * dT) >> 8);//实际温度的灵敏度
	//二阶温度补偿参考MS5611数据手册的第9页	
	if(TEMP < 2000)//当温度小于20°时二阶温度补偿
	{
		T2 = (dT * dT) >> 31;
		OFF2 = (5*(TEMP - 2000)*(TEMP - 2000)) >> 1;
		SENS2 = OFF2 >> 1;
		if(TEMP < -1500) //当温度小于-15°时
		{
			OFF2 += 7*(TEMP + 1500)*(TEMP + 1500);
			SENS2 += (11*(TEMP + 1500)*(TEMP + 1500)) >> 1;			
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;		
	}
	P = (((uncompensated_pressure_D1 * SENS) >> 21) - OFF) >> 15;//单位为Pa
	baro_temperature = TEMP *0.01f;//单位变换为°
	baro_pressure = P;
}	



float BSP_MS5611::get_Pressure(void)
{
	return baro_pressure;//返回当前压力值
}

float BSP_MS5611::get_temperature(void)
{
	return baro_temperature;//返回当前温度值
}


float BSP_MS5611::get_altitude(void)
{
	float temperature_K,presure_Pa,altitude_m;
	uint32_t ms5611_ground_pressure = 102365U;

	temperature_K = get_temperature() + 273.15f;
	presure_Pa = get_Pressure(); 
	altitude_m = 153.8462f * temperature_K * (1.0f - expf(0.190259f * logf(presure_Pa/ms5611_ground_pressure)));
	
	return altitude_m;
}


void BSP_MS5611::update(void)
{
	read_uncompensated_data();
	if(data_updated)
	{
		data_updated = false;
		calculate();
		baro_altitude = get_altitude()*100.00f;	//单位cm
	}
}

