/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_systick.h
  */

#ifndef __HAL_SYSTICK_H_
#define __HAL_SYSTICK_H_
#ifdef __cplusplus

class HAL_SYSTICK
{
public:
	//hal_systick
	void init(void);
	//time_stamp
	uint32_t millis(void);
	uint32_t micros(void);
	//delay
	void us(uint32_t delay_us);
	void ms(uint32_t delay_ms);
private:
	//time_stamp
	uint32_t ms_cnt;
	uint32_t systick_cycle;
	//delay
	uint32_t time_now;
	//hal_systick
	void get_usticks(void);
};
extern HAL_SYSTICK hal_systick;
extern HAL_SYSTICK time_stamp;
extern HAL_SYSTICK delay;


#endif
#endif







