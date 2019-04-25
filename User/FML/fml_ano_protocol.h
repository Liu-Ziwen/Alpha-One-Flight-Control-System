/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    fml_ano_protocol.h
  */ 

#ifndef __FML_ANO_PROTOCOL_H_
#define __FML_ANO_PROTOCOL_H_

/////////////////////////////////////////////////////////////////////////////////////
//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef struct
{
	uint8_t send_version;
	uint8_t send_status;
	uint8_t send_sensor;
	uint8_t send_pid1;
	uint8_t send_pid2;
	uint8_t send_pid3;
	uint8_t send_pid4;
	uint8_t send_pid5;
	uint8_t send_pid6;
	uint8_t send_rc_data;
	uint8_t send_offset;
	uint8_t send_motor_pwm;
	uint8_t send_power;
}ANO_Protocol_Structer;


#ifdef __cplusplus

class FML_ANO
{
public:
	void exchange(void);
	void receive_prepare(uint8_t data);
private:
	void send_version(uint8_t hardware_type, uint16_t hardware_ver, uint16_t software_ver, uint16_t protocol_ver, uint16_t bootloader_ver);
	void send_status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed);
	void send_sensor(int16_t a_x,int16_t a_y,int16_t a_z,int16_t g_x,int16_t g_y,int16_t g_z,int16_t m_x,int16_t m_y,int16_t m_z,int32_t bar);
	void send_rc_data(uint16_t thr,uint16_t yaw,uint16_t rol,uint16_t pit,uint16_t aux1,uint16_t aux2,uint16_t aux3,uint16_t aux4,uint16_t aux5,uint16_t aux6);
	void send_power(uint16_t votage, uint16_t current);
	void send_motor_pwm(uint16_t m_1,uint16_t m_2,uint16_t m_3,uint16_t m_4,uint16_t m_5,uint16_t m_6,uint16_t m_7,uint16_t m_8);
	void send_pid(uint8_t group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
	void send_data(uint8_t length);
	void send_check(uint8_t head, uint8_t check_sum);
	void data_receive_anl(uint8_t *data_buf,uint8_t num);
	ANO_Protocol_Structer flag;
};

extern FML_ANO fml_ano;

#endif

#endif
