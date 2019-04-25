/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    fml_ano_protocol.cpp
  */ 

#include "main.h"

FML_ANO fml_ano;

/////////////////////////////////////////////////////////////////////////////////////
//exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
//在本工程中每400Hz调用一次
void FML_ANO::exchange(void)
{
	static uint8_t cnt = 0;
	static uint8_t sensor_cnt = 2;		//200hz
	static uint8_t status_cnt = 4;		//100hz
	static uint8_t rc_data_cnt = 8;		//50hz
	static uint8_t motor_pwm_cnt = 20;	//20hz
	static uint8_t power_cnt = 40;		//10hz
	
	if((cnt % sensor_cnt) == (sensor_cnt-1))
		flag.send_sensor = 1;	
	
	if((cnt % status_cnt) == (status_cnt-1))
		flag.send_status = 1;	
	
	if((cnt % rc_data_cnt) == (rc_data_cnt-1))
		flag.send_rc_data = 1;
	
	if((cnt % motor_pwm_cnt) == (motor_pwm_cnt-1))
		flag.send_motor_pwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-1))
		flag.send_power = 1;		
	
	cnt++;
	
	
	if(flag.send_version)
	{
		flag.send_version = 0;
		send_version(4,300,100,400,0);
	}
	
	else if(flag.send_status)
	{
		flag.send_status = 0;
		send_status(0,0,0,0,0,0);
	}	
	
	else if(flag.send_sensor)
	{
		flag.send_sensor = 0;
		send_sensor(mpu6k.raw_data.ACC_X, mpu6k.raw_data.ACC_Y, mpu6k.raw_data.ACC_Z,
					mpu6k.raw_data.GYR_X, mpu6k.raw_data.GYR_Y, mpu6k.raw_data.GYR_Z,
					hmc5883l.raw_data.MAG_X, hmc5883l.raw_data.MAG_Y, hmc5883l.raw_data.MAG_Z,
					ms5611.baro_pressure);
	}	

	else if(flag.send_rc_data)
	{
		flag.send_rc_data = 0;
//		send_rc_data(PPM_CAP.Channel[2],
//					PPM_CAP.Channel[3],
//					PPM_CAP.Channel[1],
//					PPM_CAP.Channel[0],
//					PPM_CAP.Channel[4],
//					PPM_CAP.Channel[5],
//					PPM_CAP.Channel[6],
//					PPM_CAP.Channel[7],0,0);
	}	
	
	else if(flag.send_motor_pwm)
	{
		flag.send_motor_pwm = 0;
		send_motor_pwm(0,0,0,0,0,0,0,0);
		
	}	

	else if(flag.send_power)
	{
		flag.send_power = 0;
		send_power(1200,0);
	}

	else if(flag.send_pid1)
	{
		flag.send_pid1 = 0;
		send_pid(0,0,0,0,0,0,0,0,0,0);
	}	

	else if(flag.send_pid2)
	{
		flag.send_pid2 = 0;
	}

	else if(flag.send_pid3)
	{
		flag.send_pid3 = 0;
	}
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数
void FML_ANO::receive_prepare(uint8_t data)
{
	static uint8_t _data_len = 0;
	static uint8_t _data_cnt = 0;
	static uint8_t state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		com4_rx_buf[0]=data;
	}
	else if(state==1&&data==0xAF)
	{
		state=2;
		com4_rx_buf[1]=data;
	}
	else if(state==2&&data<0xF1)
	{
		state=3;
		com4_rx_buf[2]=data;
	}
	else if(state==3&&data<50)
	{
		state = 4;
		com4_rx_buf[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		com4_rx_buf[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		com4_rx_buf[4+_data_cnt]=data;
		data_receive_anl(com4_rx_buf,_data_cnt+5);
	}
	else
		state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////
//Send_Data函数是协议中所有发送数据功能使用到的发送函数
//移植时，用户应根据自身应用的情况，根据使用的通信方式，实现此函数
void FML_ANO::send_data(uint8_t length)
{
	hal_dma.transfer(COM4, length);
}

//发送校验数据给上位机
//head：帧头//check_sum：接收到的数据帧计算出的校验和
//如要返回PID1数据帧的校验数据，head = 0x10，见ANO_XIEYI的表格
void FML_ANO::send_check(uint8_t head, uint8_t check_sum)
{
	com4_tx_buf[0]=0xAA;
	com4_tx_buf[1]=0xAA;
	com4_tx_buf[2]=0xEF;
	com4_tx_buf[3]=2;
	com4_tx_buf[4]=head;
	com4_tx_buf[5]=check_sum;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<6;i++)
		sum += com4_tx_buf[i];
	com4_tx_buf[6]=sum;

	send_data(7);
}


//发送版本号
void FML_ANO::send_version(uint8_t hardware_type, uint16_t hardware_ver,uint16_t software_ver,uint16_t protocol_ver,uint16_t bootloader_ver)
{
	uint8_t _cnt=0;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x00;
	com4_tx_buf[_cnt++]=0;
	
	com4_tx_buf[_cnt++]=hardware_type;
	com4_tx_buf[_cnt++]=BYTE1(hardware_ver);
	com4_tx_buf[_cnt++]=BYTE0(hardware_ver);
	com4_tx_buf[_cnt++]=BYTE1(software_ver);
	com4_tx_buf[_cnt++]=BYTE0(software_ver);
	com4_tx_buf[_cnt++]=BYTE1(protocol_ver);
	com4_tx_buf[_cnt++]=BYTE0(protocol_ver);
	com4_tx_buf[_cnt++]=BYTE1(bootloader_ver);
	com4_tx_buf[_cnt++]=BYTE0(bootloader_ver);
	
	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	com4_tx_buf[_cnt++]=sum;
	
	send_data(_cnt);
}

//发送姿态角、高度、飞行模式、加解锁情况
void FML_ANO::send_status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	vs32 _temp2 = alt;
	
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x01;
	com4_tx_buf[_cnt++]=0;
	
	_temp = (int)(angle_rol*100);
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_pit*100);
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = (int)(angle_yaw*100);
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	
	com4_tx_buf[_cnt++]=BYTE3(_temp2);
	com4_tx_buf[_cnt++]=BYTE2(_temp2);
	com4_tx_buf[_cnt++]=BYTE1(_temp2);
	com4_tx_buf[_cnt++]=BYTE0(_temp2);
	
	com4_tx_buf[_cnt++] = fly_model;
	
	com4_tx_buf[_cnt++] = armed;
	
	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	com4_tx_buf[_cnt++]=sum;
	
	send_data(_cnt);
}

//发送传感器数据，三轴加速度计、三轴陀螺仪、三轴磁力计、气压
void FML_ANO::send_sensor(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
//	volatile int32_t _temp2;
	
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x02;
	com4_tx_buf[_cnt++]=0;
	
	_temp = a_x;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = a_y;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = a_z;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	
	_temp = g_x;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = g_y;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = g_z;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	
	_temp = m_x;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = m_y;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = m_z;	
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	
//	_temp2 = bar;
//	com4_tx_buf[_cnt++]=BYTE3(_temp2);
//	com4_tx_buf[_cnt++]=BYTE2(_temp2);
//	com4_tx_buf[_cnt++]=BYTE1(_temp2);
//	com4_tx_buf[_cnt++]=BYTE0(_temp2);
	
	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	com4_tx_buf[_cnt++] = sum;
	
	send_data(_cnt);
}

//发送遥控器信号值
void FML_ANO::send_rc_data(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6)
{
	uint8_t _cnt=0;
	
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x03;
	com4_tx_buf[_cnt++]=0;
	com4_tx_buf[_cnt++]=BYTE1(thr);
	com4_tx_buf[_cnt++]=BYTE0(thr);
	com4_tx_buf[_cnt++]=BYTE1(yaw);
	com4_tx_buf[_cnt++]=BYTE0(yaw);
	com4_tx_buf[_cnt++]=BYTE1(rol);
	com4_tx_buf[_cnt++]=BYTE0(rol);
	com4_tx_buf[_cnt++]=BYTE1(pit);
	com4_tx_buf[_cnt++]=BYTE0(pit);
	com4_tx_buf[_cnt++]=BYTE1(aux1);
	com4_tx_buf[_cnt++]=BYTE0(aux1);
	com4_tx_buf[_cnt++]=BYTE1(aux2);
	com4_tx_buf[_cnt++]=BYTE0(aux2);
	com4_tx_buf[_cnt++]=BYTE1(aux3);
	com4_tx_buf[_cnt++]=BYTE0(aux3);
	com4_tx_buf[_cnt++]=BYTE1(aux4);
	com4_tx_buf[_cnt++]=BYTE0(aux4);
	com4_tx_buf[_cnt++]=BYTE1(aux5);
	com4_tx_buf[_cnt++]=BYTE0(aux5);
	com4_tx_buf[_cnt++]=BYTE1(aux6);
	com4_tx_buf[_cnt++]=BYTE0(aux6);

	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	
	com4_tx_buf[_cnt++]=sum;
	
	send_data(_cnt);
}

//发送电压和电流值
void FML_ANO::send_power(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;
	
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x05;
	com4_tx_buf[_cnt++]=0;
	
	temp = votage;
	com4_tx_buf[_cnt++]=BYTE1(temp);
	com4_tx_buf[_cnt++]=BYTE0(temp);
	temp = current;
	com4_tx_buf[_cnt++]=BYTE1(temp);
	com4_tx_buf[_cnt++]=BYTE0(temp);
	
	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	
	com4_tx_buf[_cnt++]=sum;
	
	send_data(_cnt);
}

//发送电机PWM值
void FML_ANO::send_motor_pwm(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8)
{
	uint8_t _cnt=0;
	
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x06;
	com4_tx_buf[_cnt++]=0;
	
	com4_tx_buf[_cnt++]=BYTE1(m_1);
	com4_tx_buf[_cnt++]=BYTE0(m_1);
	com4_tx_buf[_cnt++]=BYTE1(m_2);
	com4_tx_buf[_cnt++]=BYTE0(m_2);
	com4_tx_buf[_cnt++]=BYTE1(m_3);
	com4_tx_buf[_cnt++]=BYTE0(m_3);
	com4_tx_buf[_cnt++]=BYTE1(m_4);
	com4_tx_buf[_cnt++]=BYTE0(m_4);
	com4_tx_buf[_cnt++]=BYTE1(m_5);
	com4_tx_buf[_cnt++]=BYTE0(m_5);
	com4_tx_buf[_cnt++]=BYTE1(m_6);
	com4_tx_buf[_cnt++]=BYTE0(m_6);
	com4_tx_buf[_cnt++]=BYTE1(m_7);
	com4_tx_buf[_cnt++]=BYTE0(m_7);
	com4_tx_buf[_cnt++]=BYTE1(m_8);
	com4_tx_buf[_cnt++]=BYTE0(m_8);
	
	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	
	com4_tx_buf[_cnt++]=sum;
	
	send_data(_cnt);
}

//发送PID参数
void FML_ANO::send_pid(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0xAA;
	com4_tx_buf[_cnt++]=0x10+group-1;
	com4_tx_buf[_cnt++]=0;
	
	
	_temp = p1_p * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p1_i  * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p1_d  * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p2_p  * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p2_i  * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p2_d * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p3_p  * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p3_i  * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	_temp = p3_d * 1000;
	com4_tx_buf[_cnt++]=BYTE1(_temp);
	com4_tx_buf[_cnt++]=BYTE0(_temp);
	
	com4_tx_buf[3] = _cnt-4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += com4_tx_buf[i];
	
	com4_tx_buf[_cnt++]=sum;

	send_data(_cnt);
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
void FML_ANO::data_receive_anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	for(uint8_t i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==0x01)
	{
		if(*(data_buf+4)==0x01)
		{
//			mpu6050.Acc_CALIBRATE = 1;//校准加速度计
		}
		if(*(data_buf+4)==0x02)
		{
//			mpu6050.Gyro_CALIBRATE = 1;//校准陀螺仪
		}
		if(*(data_buf+4)==0x03)
		{
//			mpu6050.Acc_CALIBRATE = 1;		
//			mpu6050.Gyro_CALIBRATE = 1;			
		}
	}
	
	if(*(data_buf+2)==0x02)
	{
		if(*(data_buf+4)==0x01)
		{
			flag.send_pid1 = 1;
			flag.send_pid2 = 1;
			flag.send_pid3 = 1;
			flag.send_pid4 = 1;
			flag.send_pid5 = 1;
			flag.send_pid6 = 1;
		}
		if(*(data_buf+4)==0x02)
		{
			
		}
		if(*(data_buf+4)==0xA0)		//读取版本信息
		{
			flag.send_version = 1;
		}
		if(*(data_buf+4)==0xA1)		//恢复默认参数
		{
//			Para_ResetToFactorySetup();
		}
	}

	if(*(data_buf+2)==0x10)								//PID1
    {
//        ctrl_1.PID[PIDROLL].kp  = 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
//        ctrl_1.PID[PIDROLL].ki  = 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
//        ctrl_1.PID[PIDROLL].kd  = 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ctrl_1.PID[PIDPITCH].kp = 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ctrl_1.PID[PIDPITCH].ki = 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ctrl_1.PID[PIDPITCH].kd = 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_1.PID[PIDYAW].kp   = 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_1.PID[PIDYAW].ki   = 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_1.PID[PIDYAW].kd   = 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        send_check(*(data_buf+2),sum);
//				Param_SavePID();
    }
    if(*(data_buf+2)==0x11)								//PID2
    {
//        ctrl_1.PID[PID4].kp 	= 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
//        ctrl_1.PID[PID4].ki 	= 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
//        ctrl_1.PID[PID4].kd 	= 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ctrl_1.PID[PID5].kp 	= 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ctrl_1.PID[PID5].ki 	= 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ctrl_1.PID[PID5].kd 	= 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_1.PID[PID6].kp	= 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_1.PID[PID6].ki 	= 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_1.PID[PID6].kd 	= 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        send_check(*(data_buf+2),sum);
//				Param_SavePID();
    }
    if(*(data_buf+2)==0x12)								//PID3
    {	
//        ctrl_2.PID[PIDROLL].kp  = 0.001*( (volatile int16_t)(*(data_buf+4)<<8)|*(data_buf+5) );
//        ctrl_2.PID[PIDROLL].ki  = 0.001*( (volatile int16_t)(*(data_buf+6)<<8)|*(data_buf+7) );
//        ctrl_2.PID[PIDROLL].kd  = 0.001*( (volatile int16_t)(*(data_buf+8)<<8)|*(data_buf+9) );
//        ctrl_2.PID[PIDPITCH].kp = 0.001*( (volatile int16_t)(*(data_buf+10)<<8)|*(data_buf+11) );
//        ctrl_2.PID[PIDPITCH].ki = 0.001*( (volatile int16_t)(*(data_buf+12)<<8)|*(data_buf+13) );
//        ctrl_2.PID[PIDPITCH].kd = 0.001*( (volatile int16_t)(*(data_buf+14)<<8)|*(data_buf+15) );
//        ctrl_2.PID[PIDYAW].kp   = 0.001*( (volatile int16_t)(*(data_buf+16)<<8)|*(data_buf+17) );
//        ctrl_2.PID[PIDYAW].ki   = 0.001*( (volatile int16_t)(*(data_buf+18)<<8)|*(data_buf+19) );
//        ctrl_2.PID[PIDYAW].kd   = 0.001*( (volatile int16_t)(*(data_buf+20)<<8)|*(data_buf+21) );
        send_check(*(data_buf+2),sum);
//				Param_SavePID();
    }
	if(*(data_buf+2)==0x13)								//PID4
	{
		send_check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0x14)								//PID5
	{
		send_check(*(data_buf+2),sum);
	}
	if(*(data_buf+2)==0x15)								//PID6
	{
		send_check(*(data_buf+2),sum);
	}
}




