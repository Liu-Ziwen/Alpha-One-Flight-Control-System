/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    bsp_ubloxm8n.cpp
  */ 

#include "main.h"

BSP_UBLOXM8N ublox_m8n;

uint8_t  UBLOX_SET_BINARY[]= "\265\142\006\001\003\000\001\006\001\022\117$PUBX,41,1,0003,0001,38400,0*26\r\n";


void BSP_UBLOXM8N::init(void)
{
	send_info();
	while(com5_tx_over==false);
	hal_uart.com5_init(38400);
	while(com5_tx_over==false);
	config_navigation_rate(200);
	while(com5_tx_over==false);
	config_message_rate(CLASS_NAV, MSG_POSLLH, 1);
	while(com5_tx_over==false);
	config_message_rate(CLASS_NAV, MSG_STATUS, 1);
	while(com5_tx_over==false);
	config_message_rate(CLASS_NAV, MSG_SOL, 1);
	while(com5_tx_over==false);
	config_message_rate(CLASS_NAV, MSG_VELNED, 1);
	while(com5_tx_over==false);
	config_message_rate(CLASS_MON, MSG_MON_HW, 2);
	while(com5_tx_over==false);
	config_message_rate(CLASS_MON, MSG_MON_HW2, 2);
	while(com5_tx_over==false);	
//	Quene_init(&Gps_quene,GPS_QUENE_SIZE);
}


void BSP_UBLOXM8N::update_checksum(uint8_t *data, uint8_t length, uint8_t *ck_a, uint8_t *ck_b)
{
    while (length--)
	{
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}

 
void BSP_UBLOXM8N::send_info(void)
{
	int x;
  for(x=0;x<sizeof(UBLOX_SET_BINARY);x++)
	{
		com5_tx_buf[x]=UBLOX_SET_BINARY[x];
	}	
	hal_dma.transfer(COM5, COM5_TX_BUFFER_SIZE); 
}

 
void BSP_UBLOXM8N::send_message(uint8_t msg_class, uint8_t msg_id, void *msg, uint8_t size)
{
    ubx_header_structer header;
	uint8_t x;
	uint8_t idx;
    uint8_t ck_a=0, ck_b=0;
    header.preamble1 = 0XB5;
    header.preamble2 = 0X62;
    header.msg_class = msg_class;
    header.msg_id    = msg_id;
    header.length    = size;

    update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, &ck_a, &ck_b);
    update_checksum((uint8_t *)msg, size, &ck_a, &ck_b);
		
	for(x=0; x<COM5_TX_BUFFER_SIZE; x++)
	{						
		com5_tx_buf[x]= 0;
	}	
	
	idx=0;
	for(x=0; x<sizeof(header); x++)
	{						
		com5_tx_buf[x]=  *(((uint8_t *)&header)+idx);
		idx++; // idx 为header坐标，下面为msg坐标
	}	
	idx=0;
	for( ; x<sizeof(header)+size; x++)
	{			
		com5_tx_buf[x]=  *(((uint8_t *)msg)+idx);
		idx++;
	}			
	com5_tx_buf[x++]=ck_a;
	com5_tx_buf[x++]=ck_b;
	hal_dma.transfer(COM5, COM5_TX_BUFFER_SIZE);
}

void BSP_UBLOXM8N::config_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    ubx_cfg_msg_rate_structer msg;
    msg.msg_class = msg_class;
    msg.msg_id    = msg_id;
    msg.rate      = rate;
    send_message(0x06, 0x01, &msg, sizeof(msg));
}

 void BSP_UBLOXM8N::config_navigation_rate(uint16_t rate_ms)
{
    ubx_cfg_nav_rate_structer msg;
    msg.measure_rate_ms = rate_ms;
    msg.nav_rate        = 1;
    msg.timeref         = 0;     // UTC time
    send_message(0x06, 0x08, &msg, sizeof(msg));
}











