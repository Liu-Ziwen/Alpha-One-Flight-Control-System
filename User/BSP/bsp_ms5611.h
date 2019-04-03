/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    bsp_ms5611.h
  */

#ifndef __BSP_MS5611_H_
#define __BSP_MS5611_H_


#define MS5611_DISABLE 	GPIOB->BSRRL = GPIO_Pin_15	//GPIO_SetBits(GPIOB,GPIO_Pin_15)
#define MS5611_ENABLE 	GPIOB->BSRRH = GPIO_Pin_15	//GPIO_ResetBits(GPIOB,GPIO_Pin_15)

#define CMD_MS5611_RESET 		0x1E
#define CMD_MS5611_PROM_Setup 	0xA0
#define CMD_MS5611_PROM_C1 		0xA2
#define CMD_MS5611_PROM_C2 		0xA4
#define CMD_MS5611_PROM_C3 		0xA6
#define CMD_MS5611_PROM_C4 		0xA8
#define CMD_MS5611_PROM_C5 		0xAA
#define CMD_MS5611_PROM_C6 		0xAC
#define CMD_MS5611_PROM_CRC 	0xAE
#define CMD_CONVERT_D1_OSR4096 	0x48   // Maximum resolution (oversampling)
#define CMD_CONVERT_D2_OSR4096 	0x58   // Maximum resolution (oversampling)


#ifdef __cplusplus

class BSP_MS5611
{
public:
	float baro_pressure;				//计算后的压力，单位为Pa
	float baro_temperature;				//计算后的温度，单位为℃
	float baro_altitude;				//根据压力和温度换算出来的高度，换算后单位m，乘以100为cm
	bool data_updated;
	void init(void);
	void update(void);

private:
	//6个出厂校准系数，Calibration Coefficient
	uint16_t calibration_c1_SENS_T1;	//压力灵敏度				Pressure sensitivity
	uint16_t calibration_c2_OFF_T1;		//压力零偏					Pressure offset
	uint16_t calibration_c3_TCS;		//压力灵敏度的温度矫正系数	Temperature coefficient of pressure sensitivity
	uint16_t calibration_c4_TCO;		//压力零偏的温度校正系数	Temperature coefficient of pressure offset
	uint16_t calibration_c5_T_REF;		//参考温度					Reference temperature
	uint16_t calibration_c6_TEMPSENS;	//温度的温度校准系数		Temperature coefficient of the temperature

	uint8_t stage;						//读取阶段
	uint32_t uncompensated_pressure_D1;	//原始压力数据，发指令CMD_CONVERT_D1_OSR4096读出
	uint32_t uncompensated_temp_D2;		//原始温度数据，发指令CMD_CONVERT_D2_OSR4096读出
	
	void init_cs(void);
	void send_cmd(uint8_t data);
	uint16_t read_two_bytes(uint8_t addr);
	void get_calibration_coeff(void);
	uint32_t read_ADC(void);
	void read_uncompensated_data(void);
	void calculate(void);
	float get_Pressure(void);
	float get_temperature(void);
	float get_altitude(void);
};
extern BSP_MS5611 ms5611;


#endif

#endif


