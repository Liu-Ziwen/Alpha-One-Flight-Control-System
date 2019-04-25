/*
	Global_Variables
*/
#include "main.h"


/* AHRS姿态航向参考系统--------------------------------------------------------------------------------------------------- */
IMU_SensorData_Structer	IMU_SensorData;				//IMU数据结构体(陀螺仪和加速度计)//由6轴传感器调整方向及零偏后获得
IMU_SensorData_Structer IMU_Offset;

Compass_SensorData_Structer Compass_SensorData;		//磁罗盘数据结构体
GPS_SensorData_Structer Gps_Data;


/* COM串口DMA收发缓冲区--------------------------------------------------------------------------------------------------- */
//uint8_t COM4_Tx_Buffer[COM4_TX_BUFFER_SIZE];
//uint8_t COM4_Rx_Buffer[COM4_RX_BUFFER_SIZE];

//uint8_t COM1_Tx_Buffer[COM1_TX_BUFFER_SIZE];
//uint8_t COM1_Rx_Buffer[COM1_RX_BUFFER_SIZE];




