/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    com_global.h
  */

#ifndef __COM_GLOBAL_H_
#define __COM_GLOBAL_H_
/* 常用常数定义----------------------------------------------------------------------------------------------------------- */
#define GRAVITY_MSS 9.80665f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f
#define PI 3.141592653589793238462643383279f


/* AHRS姿态航向参考系统--------------------------------------------------------------------------------------------------- */
typedef struct
{
	float ACC_X;	
	float ACC_Y;
	float ACC_Z;
	float IMU_TEMP;		//MPU读温度的
	float GYR_X;
	float GYR_Y;
	float GYR_Z;	
}IMU_SensorData_Structer;//IMU数据结构体（陀螺仪和加速度计）//由6轴传感器调整方向及零偏后获得
extern IMU_SensorData_Structer IMU_SensorData;
extern IMU_SensorData_Structer IMU_Offset;


typedef struct 
{
	float MAG_X;
	float MAG_Y;
	float MAG_Z;
}Compass_SensorData_Structer;//磁罗盘数据结构体
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



/* COM串口DMA收发缓冲区--------------------------------------------------------------------------------------------------- */
//#define COM4_TX_BUFFER_SIZE 	100	//串口4发送数据长度(数传和地面站)
//#define COM4_RX_BUFFER_SIZE 	50	//串口4接收数据长度

//#define COM1_TX_BUFFER_SIZE 	50	//串口1发送数据长度(GPS)
//#define COM1_RX_BUFFER_SIZE 	1	//串口1接收数据长度

//extern uint8_t COM4_Tx_Buffer[COM4_TX_BUFFER_SIZE];
//extern uint8_t COM4_Rx_Buffer[COM4_RX_BUFFER_SIZE];

//extern uint8_t COM1_Tx_Buffer[COM1_TX_BUFFER_SIZE];
//extern uint8_t COM1_Rx_Buffer[COM1_RX_BUFFER_SIZE];



#ifdef __cplusplus

#endif

#endif


