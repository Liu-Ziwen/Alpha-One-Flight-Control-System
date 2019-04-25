/**
  ******************************************************************************
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    12-April-2019
  * @brief   com_test.cpp
  ******************************************************************************
**/	

#include "main.h"

/**
  ******************************************************************************
  * @test		 Optical Flow Module 
  * @author  Ziwen LIU
  * @date    18-April-2019
  ******************************************************************************
**/
const static uint8_t sensor_cfg[]={
//��ַ, ����
  0x12, 0x80, 
  0x11, 0x30, 
  0x1b, 0x06, 
  0x6b, 0x43, 
  0x12, 0x20, 
  0x3a, 0x00, 
  0x15, 0x02, 
  0x62, 0x81, 
  0x08, 0xa0, 
  0x06, 0x68, 
  0x2b, 0x20, 
  0x92, 0x25, 
  0x27, 0x97, 
  0x17, 0x01, 
  0x18, 0x79, 
  0x19, 0x00, 
  0x1a, 0xa0, 
  0x03, 0x00, 
  0x13, 0x00, 
  0x01, 0x13, 
  0x02, 0x20, 
  0x87, 0x16, 
  0x8c, 0x01, 
  0x8d, 0xcc, 
  0x13, 0x07, 
  0x33, 0x10, 
  0x34, 0x1d, 
  0x35, 0x46, 
  0x36, 0x40, 
  0x37, 0xa4, 
  0x38, 0x7c, 
  0x65, 0x46, 
  0x66, 0x46, 
  0x6e, 0x20, 
  0x9b, 0xa4, 
  0x9c, 0x7c, 
  0xbc, 0x0c, 
  0xbd, 0xa4, 
  0xbe, 0x7c, 
  0x20, 0x09, 
  0x09, 0x03, 
  0x72, 0x2f, 
  0x73, 0x2f, 
  0x74, 0xa7, 
  0x75, 0x12, 
  0x79, 0x8d, 
  0x7a, 0x00, 
  0x7e, 0xfa, 
  0x70, 0x0f, 
  0x7c, 0x84, 
  0x7d, 0xba, 
  0x5b, 0xc2, 
  0x76, 0x90, 
  0x7b, 0x55, 
  0x71, 0x46, 
  0x77, 0xdd, 
  0x13, 0x0f, 
  0x8a, 0x10, 
  0x8b, 0x20, 
  0x8e, 0x21, 
  0x8f, 0x40, 
  0x94, 0x41, 
  0x95, 0x7e, 
  0x96, 0x7f, 
  0x97, 0xf3, 
  0x13, 0x07, 
  0x24, 0x58, 
  0x97, 0x48, 
  0x25, 0x08, 
  0x94, 0xb5, 
  0x95, 0xc0, 
  0x80, 0xf4, 
  0x81, 0xe0, 
  0x82, 0x1b, 
  0x83, 0x37, 
  0x84, 0x39, 
  0x85, 0x58, 
  0x86, 0xff, 
  0x89, 0x15, 
  0x8a, 0xb8, 
  0x8b, 0x99, 
  0x39, 0x98, 
  0x3f, 0x98, 
  0x90, 0xa0, 
  0x91, 0xe0, 
  0x40, 0x20, 
  0x41, 0x28, 
  0x42, 0x26, 
  0x43, 0x25, 
  0x44, 0x1f, 
  0x45, 0x1a, 
  0x46, 0x16, 
  0x47, 0x12, 
  0x48, 0x0f, 
  0x49, 0x0d, 
  0x4b, 0x0b, 
  0x4c, 0x0a, 
  0x4e, 0x08, 
  0x4f, 0x06, 
  0x50, 0x06, 
  0x5a, 0x56, 
  0x51, 0x1b, 
  0x52, 0x04, 
  0x53, 0x4a, 
  0x54, 0x26, 
  0x57, 0x75, 
  0x58, 0x2b, 
  0x5a, 0xd6, 
  0x51, 0x28, 
  0x52, 0x1e, 
  0x53, 0x9e, 
  0x54, 0x70, 
  0x57, 0x50, 
  0x58, 0x07, 
  0x5c, 0x28, 
  0xb0, 0xe0, 
  0xb1, 0xc0, 
  0xb2, 0xb0, 
  0xb3, 0x4f, 
  0xb4, 0x63, 
  0xb4, 0xe3, 
  0xb1, 0xf0, 
  0xb2, 0xa0, 
  0x55, 0x00, 
  0x56, 0x40, 
  0x96, 0x50, 
  0x9a, 0x30, 
  0x6a, 0x81, 
  0x23, 0x33, 
  0xa0, 0xd0, 
  0xa1, 0x31, 
  0xa6, 0x04, 
  0xa2, 0x0f, 
  0xa3, 0x2b, 
  0xa4, 0x0f, 
  0xa5, 0x2b, 
  0xa7, 0x9a, 
  0xa8, 0x1c, 
  0xa9, 0x11, 
  0xaa, 0x16, 
  0xab, 0x16, 
  0xac, 0x3c, 
  0xad, 0xf0, 
  0xae, 0x57, 
  0xc6, 0xaa, 
  0xd2, 0x78, 
  0xd0, 0xb4, 
  0xd1, 0x00, 
  0xc8, 0x10, 
  0xc9, 0x12, 
  0xd3, 0x09, 
  0xd4, 0x2a, 
  0xee, 0x4c, 
  0x7e, 0xfa, 
  0x74, 0xa7, 
  0x78, 0x4e, 
  0x60, 0xe7, 
  0x61, 0xc8, 
  0x6d, 0x70, 
  0x1e, 0x39, 
  0x98, 0x1a
};

TEST_OPTICAL_FLOW optical_flow_module;

bool test_flag=false;


bool TEST_OPTICAL_FLOW::init(void)
{
	//��ʼ��1���ϵ�
	hal_uart.com4_send_byte(0xAA);//��������ָ�ģ���޷���
	delay.ms(10);
	//��ʼ��2��ģ���ڲ��������ã��ɹ������յ�0xAB��0x00��0xAB
	hal_uart.com4_send_byte(0xAB);//ģ���ڲ���������ָ�ģ�鷵��0xAB
	hal_uart.com4_send_byte(0x96);
	hal_uart.com4_send_byte(0x26);
	hal_uart.com4_send_byte(0xBC);
	hal_uart.com4_send_byte(0x50);//�ĸ��ڲ�����ָ�ȫ���������ģ�鷵��0x00�ɹ���0x01��Ӧ��0x02У�����
	hal_uart.com4_send_byte(0x5C);//�ĸ��ڲ�����ָ������ֵ��ΪXORУ��
	delay.ms(10);
	if((test_temp[0]!=0xAB)||(test_temp[1]!=0x00)||(test_temp[2]!=0xAB))
		return false;
	//��ʼ��3��
	uint16_t length;
	length=sizeof(sensor_cfg);
	for(uint16_t test_i=0; test_i<length; test_i+=2)
	{
		static uint16_t j=0;
		j+=3;
		stage3_init(sensor_cfg[test_i], sensor_cfg[test_i+1]);
		delay.ms(10);
		if((test_temp[j]!=0xBB)||(test_temp[j+1]!=0x00)||(test_temp[j+2]!=0xBB))
			return false;
	}
	//��ʼ��4��
	hal_uart.com4_send_byte(0xDD);//�ر�����ָ�ģ���޷���ֵ
	return true;
}	

void TEST_OPTICAL_FLOW::stage3_init(uint8_t address, uint8_t data)
{
	hal_uart.com4_send_byte(0xBB);//��������������ָ�ģ�鷵��0xBB
	hal_uart.com4_send_byte(0xDC);
	hal_uart.com4_send_byte(address);
	hal_uart.com4_send_byte(data);
	hal_uart.com4_send_byte(0xDC^address^data);//�ĸ��ڲ�����ָ�ȫ���������ģ�鷵��0x00�ɹ���0x01��Ӧ��0x02У�����
	
}


uint8_t optical_flow_data[14]={0};

bool optical_flow_module_data_protocol(uint8_t rcvdata)
{
	static uint8_t rcvPart = 0;
	static uint8_t rcvDataNo = 0;
	static uint8_t rcvDataLen = 0;
	static uint8_t rcvingBuf[13];

	switch(rcvPart)
	{
			case 0:
				if(rcvdata == 0xFE)
				{
					rcvPart = 1;
					rcvingBuf[rcvDataNo] = 0xFE;
					rcvDataNo++;
				}
				break;
			
			case 1:
				if(rcvdata == 0x0A)
				{
					rcvPart =2;
					rcvingBuf[rcvDataNo] = 0x0A;
					rcvDataNo++;
				}
				else
				{
					rcvPart = 0;
					rcvDataNo =0;
				}
				break;

			case 2:
				rcvDataLen = 12;//rcvDataLenָ���ǳ�����������ͷ֮��ģ��������ݳ���
				rcvPart = 3;
				rcvingBuf[rcvDataNo] = rcvdata;
				rcvDataNo++;
				break;
			
			case 3:
				rcvingBuf[rcvDataNo] = rcvdata;
				rcvDataNo++;
				rcvDataLen--;
				if(rcvDataLen == 0)
				{
					rcvPart = 4;
				}
				break;

			case 4:
				for(uint8_t k = 0 ;k < 14 ; k++)
				{
					optical_flow_data[k] = rcvingBuf[k];
				}
				optical_flow_module_data_handle();
				rcvPart = 0;
				rcvDataNo = 0;
				return true;

			default:
				break;
		}
	return false;
}

int16_t flow_x_integral=0;
int16_t flow_y_integral=0;
uint16_t integration_timespan=0;
uint16_t ground_distance=0;
uint8_t valid=0;
uint8_t version=0;


void optical_flow_module_data_handle(void)
{
	uint8_t _xor=0;
	for(uint8_t i=2;i<12;i++)_xor^=optical_flow_data[i];
	if(_xor!=optical_flow_data[12])return;
	
	else
	{
		flow_x_integral=COM_Joint_Two8Bit_To_16Bit(optical_flow_data[3],optical_flow_data[2]);
		flow_y_integral=COM_Joint_Two8Bit_To_16Bit(optical_flow_data[5],optical_flow_data[4]);
		integration_timespan=COM_Joint_Two8Bit_To_16Bit(optical_flow_data[7],optical_flow_data[6]);
		ground_distance=COM_Joint_Two8Bit_To_16Bit(optical_flow_data[9],optical_flow_data[8]);
		valid=optical_flow_data[10];
		version=optical_flow_data[11];
	}
}


