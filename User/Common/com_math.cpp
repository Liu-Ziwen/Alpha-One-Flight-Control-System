#include "main.h"

//把高8位和低8位合并成一个16位的数
int16_t COM_Joint_Two8Bit_To_16Bit(uint8_t Byte_High,uint8_t Byte_Low)
{
	return ((Byte_High << 8) + Byte_Low);
}

