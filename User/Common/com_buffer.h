/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    22-April-2019
  * @file    com_buffer.h
  */

#ifndef __COM_BUFFER_H_
#define __COM_BUFFER_H_

#define GPS_QUENE_SIZE 500

typedef struct
{
	uint16_t quene[GPS_QUENE_SIZE];
	uint16_t head;
	uint16_t tail;
	uint16_t size;
	uint16_t count;
}Quene_u8_Structer;

extern Quene_u8_Structer gps_quene;


#ifdef __cplusplus





#endif

#endif

