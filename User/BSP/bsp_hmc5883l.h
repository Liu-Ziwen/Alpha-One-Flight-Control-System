/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    bsp_hmc5883l.h
  */

#ifndef __BSP_HMC5883L_H_
#define __BSP_HMC5883L_H_

//����������IIC������HMC5883L
#define HMC5883L_SLAVE_ADDR		0x1E	//7-bit��ַ
#define	HMC5883L_WRITE_ADDR   	0x3C	//8-bitд���ַ
#define	HMC5883L_READ_ADDR   	0x3D	//8-bit��ȡ��ַ

#define HMC5883L_R_CONFA 		0x00  	//���üĴ���A��ַ
#define HMC5883L_R_CONFB 		0x01  	//���üĴ���B��ַ
#define HMC5883L_R_MODE  		0x02  	//����ģʽ��ַ

#define HMC5883L_R_XMSB			0x03	//��ȡ�Ĵ���
#define HMC5883L_R_STATUS		0x09	//״̬�Ĵ���
#define HMC5883L_R_IDA			0x0A	//IDA��B��C


typedef struct 
{
	int16_t MAG_X;
	int16_t MAG_Y;
	int16_t MAG_Z;
}HMC5883L_SensorData_Raw_Structer;

#ifdef __cplusplus

class BSP_HMC5883L
{
public:
	bool have_compass;
	HMC5883L_SensorData_Raw_Structer raw_data;
	void init_with_check(void);
	void update(void);	
private:
	void write_reg(uint8_t addr, uint8_t data);
	uint8_t read_reg(uint8_t addr);
	void read_len(uint8_t addr, uint8_t len, uint8_t *data);
	void init(void);
	bool check_WHOAMI(void);

};

extern BSP_HMC5883L hmc5883l;

#endif

#endif
