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
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//ʹ��GPIOʱ��
	//��CSƬѡ�źŵ����Խ�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;//PB15
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHZ
	GPIO_Init(GPIOB,&GPIO_InitStructure);//��ʼ��
	MS5611_DISABLE;//Ƭѡ�����ߣ���ѡ��
}

//��MS5611����һ��ָ��
void BSP_MS5611::send_cmd(uint8_t data)
{
	MS5611_ENABLE;
	hal_spi.rw_byte(data);
	MS5611_DISABLE;
}

//�Ӹ����Ĵ�����ַ��ȡ��������
uint16_t BSP_MS5611::read_two_bytes(uint8_t addr)
{
	uint8_t data[2];
	MS5611_ENABLE;
	hal_spi.read_multi_reg(addr,2,data);
	MS5611_DISABLE;
	return (((uint32_t)data[0])<<8) | ((uint32_t)data[1]);
}

//��ȡMS5611�ĳ���У׼ϵ��
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

//��ȡMS5611�Ĵ��������ݣ���ȡѹ�������¶���ָ�����
uint32_t BSP_MS5611::read_ADC(void)
{
	uint8_t data[3];
	MS5611_ENABLE;
	hal_spi.read_multi_reg(0,3,data);
	MS5611_DISABLE;
	return (((uint32_t)data[0])<<16) | (((uint32_t)data[1])<<8) | ((uint32_t)data[2]);
}

//��MS5611���г�ʼ��
void BSP_MS5611::init(void)
{
	init_cs();
	send_cmd(CMD_MS5611_RESET);			//��MS5611���͸�λָ��
	delay.ms(100);
	get_calibration_coeff();			//��ȡ����У׼ϵ��
	delay.ms(1);
}


////���������ȡδ������ԭʼ����
void BSP_MS5611::read_uncompensated_data(void)
{
	if(stage == 0) 									//��ȡ�¶ȴ���������
	{
		uncompensated_temp_D2 = read_ADC();			//��ȡһ���¶�����
		stage++;
		send_cmd(CMD_CONVERT_D1_OSR4096);			//����ѹ������ת��ָ��
	}
	else
	{
		uncompensated_pressure_D1 = read_ADC();		//��ȡһ��ѹ������
		data_updated = true;
		stage++;	
		if(stage == 5)
		{
			send_cmd(CMD_CONVERT_D2_OSR4096);		//����ζ�ȡ�¶ȴ�����ֵ
			stage = 0;
		}
		else
		{
			send_cmd(CMD_CONVERT_D1_OSR4096);		//�����Ĵζ�ȡѹ��������ֵ
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
	//���㹫ʽ�ο�MS5611�����ֲ�ĵ�8ҳ
	dT = uncompensated_temp_D2 - ((int32_t)calibration_c5_T_REF  << 8);//ʵ�ʺͲο��¶�֮��Ĳ�
	TEMP = 2000 + ((dT * calibration_c6_TEMPSENS) >> 23);//ʵ���¶�
	OFF = ((uint32_t)calibration_c2_OFF_T1 << 16) + ((calibration_c4_TCO * dT) >> 7);//ʵ���¶ȵ���ƫ
	SENS = ((uint32_t)calibration_c1_SENS_T1 << 15) +((calibration_c3_TCS * dT) >> 8);//ʵ���¶ȵ�������
	//�����¶Ȳ����ο�MS5611�����ֲ�ĵ�9ҳ	
	if(TEMP < 2000)//���¶�С��20��ʱ�����¶Ȳ���
	{
		T2 = (dT * dT) >> 31;
		OFF2 = (5*(TEMP - 2000)*(TEMP - 2000)) >> 1;
		SENS2 = OFF2 >> 1;
		if(TEMP < -1500) //���¶�С��-15��ʱ
		{
			OFF2 += 7*(TEMP + 1500)*(TEMP + 1500);
			SENS2 += (11*(TEMP + 1500)*(TEMP + 1500)) >> 1;			
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;		
	}
	P = (((uncompensated_pressure_D1 * SENS) >> 21) - OFF) >> 15;//��λΪPa
	baro_temperature = TEMP *0.01f;//��λ�任Ϊ��
	baro_pressure = P;
}	



float BSP_MS5611::get_Pressure(void)
{
	return baro_pressure;//���ص�ǰѹ��ֵ
}

float BSP_MS5611::get_temperature(void)
{
	return baro_temperature;//���ص�ǰ�¶�ֵ
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
		baro_altitude = get_altitude()*100.00f;	//��λcm
	}
}

