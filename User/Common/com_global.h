/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    com_global.h
  */

#ifndef __COM_GLOBAL_H_
#define __COM_GLOBAL_H_
/* ���ó�������----------------------------------------------------------------------------------------------------------- */
#define GRAVITY_MSS 9.80665f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define PI 3.141592653589793238462643383279f


/* AHRS��̬����ο�ϵͳ--------------------------------------------------------------------------------------------------- */
typedef struct
{
	float ACC_X;	
	float ACC_Y;
	float ACC_Z;
	float IMU_TEMP;		//MPU���¶ȵ�
	float GYR_X;
	float GYR_Y;
	float GYR_Z;	
}IMU_SensorData_Structer;//IMU���ݽṹ�壨�����Ǻͼ��ٶȼƣ�//��6�ᴫ��������������ƫ����
extern IMU_SensorData_Structer IMU_SensorData;
extern IMU_SensorData_Structer IMU_Offset;


typedef struct 
{
	float MAG_X;
	float MAG_Y;
	float MAG_Z;
}Compass_SensorData_Structer;//���������ݽṹ��
extern Compass_SensorData_Structer Compass_SensorData;

typedef struct
{
	int32_t	Lattitude;
	int32_t	Longitude;
	uint8_t FixSta;
	uint8_t SVN;
	int32_t	North_vector;
	int32_t	East_vector;
	int32_t	Down_vector;
	int32_t speed_3d_cm;
	int32_t ground_speed_cm;
	uint32_t last_fix_time;
	int16_t hdop;
	bool updated;
}GPS_SensorData_Structer;
extern GPS_SensorData_Structer Gps_Data;



/* COM����DMA�շ�������--------------------------------------------------------------------------------------------------- */
//#define COM4_TX_BUFFER_SIZE 	100	//����4�������ݳ���(�����͵���վ)
//#define COM4_RX_BUFFER_SIZE 	50	//����4�������ݳ���

//#define COM1_TX_BUFFER_SIZE 	50	//����1�������ݳ���(GPS)
//#define COM1_RX_BUFFER_SIZE 	1	//����1�������ݳ���

//extern uint8_t COM4_Tx_Buffer[COM4_TX_BUFFER_SIZE];
//extern uint8_t COM4_Rx_Buffer[COM4_RX_BUFFER_SIZE];

//extern uint8_t COM1_Tx_Buffer[COM1_TX_BUFFER_SIZE];
//extern uint8_t COM1_Rx_Buffer[COM1_RX_BUFFER_SIZE];



#ifdef __cplusplus

#endif

#endif


