#include "main.h"

//�Ѹ�8λ�͵�8λ�ϲ���һ��16λ����
int16_t COM_Joint_Two8Bit_To_16Bit(uint8_t Byte_High,uint8_t Byte_Low)
{
	return ((Byte_High << 8) + Byte_Low);
}

