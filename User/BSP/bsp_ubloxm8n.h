/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    bsp_ubloxm8n.h
  */ 

#ifndef __BSP_UBLOXM8N_H_
#define __BSP_UBLOXM8N_H_

typedef struct
{
	uint8_t preamble1;
	uint8_t preamble2;
	uint8_t msg_class;
	uint8_t msg_id;
	uint16_t length;
}ubx_header_structer;

typedef struct  
{
	uint8_t msg_class;
	uint8_t msg_id;
	uint8_t rate;
}ubx_cfg_msg_rate_structer;

typedef struct   
{
	uint16_t measure_rate_ms;
	uint16_t nav_rate;
	uint16_t timeref;
}ubx_cfg_nav_rate_structer;

typedef enum  
{
		PREAMBLE1 = 0xb5,
		PREAMBLE2 = 0x62,
		CLASS_NAV = 0x01,
		CLASS_ACK = 0x05,
		CLASS_CFG = 0x06,
		CLASS_MON = 0x0A,
		MSG_ACK_NACK = 0x00,
		MSG_ACK_ACK = 0x01,
		MSG_POSLLH = 0x2,
		MSG_STATUS = 0x3,
		MSG_SOL = 0x6,
		MSG_VELNED = 0x12,
		MSG_CFG_PRT = 0x00,
		MSG_CFG_RATE = 0x08,
		MSG_CFG_SET_RATE = 0x01,
		MSG_CFG_NAV_SETTINGS = 0x24,
		MSG_MON_HW = 0x09,
		MSG_MON_HW2 = 0x0B
}ubs_protocol_bytes;

#ifdef __cplusplus

class BSP_UBLOXM8N
{
public:
	void init(void);
	void re_init(void);//用于飞行中重新初始化
	void data_anl(uint8_t data);
private:
	void update_checksum(uint8_t *data, uint8_t length, uint8_t *ck_a, uint8_t *ck_b);
	void send_info(void);
	void send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size);
	void config_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate);
	void config_navigation_rate(uint16_t rate_ms);
	
};

extern BSP_UBLOXM8N ublox_m8n;

#endif

#endif


