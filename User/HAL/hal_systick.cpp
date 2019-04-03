/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_systick.cpp
  */

#include "main.h"

static volatile uint32_t usticks = 0;
static volatile uint32_t system_real_time_ms = 0;

HAL_SYSTICK hal_systick;
HAL_SYSTICK time_stamp;
HAL_SYSTICK delay;


void HAL_SYSTICK::init(void)
{
	get_usticks();
	SysTick_Config(SystemCoreClock / 1000U);
}


void HAL_SYSTICK::get_usticks(void)
{
	RCC_ClocksTypeDef RCC_ClocksStructure;
	RCC_GetClocksFreq(&RCC_ClocksStructure);
	
	usticks = RCC_ClocksStructure.SYSCLK_Frequency / 1000000;
}

//毫秒计时产生时间戳//TimeStamp
uint32_t HAL_SYSTICK::millis(void)
{
	return system_real_time_ms;
}

//微秒计时产生时间戳//TimeStamp
uint32_t HAL_SYSTICK::micros(void)
{
	do
	{
		ms_cnt = system_real_time_ms;
		systick_cycle = SysTick->VAL;
	}while(ms_cnt != system_real_time_ms);
	//毫秒加上已计时的微秒为总的微秒数
	return (ms_cnt * 1000) + (usticks * 1000 - systick_cycle) / usticks;
}

//延时函数，单位为微秒//Delay
void HAL_SYSTICK::us(uint32_t delay_us)
{
	time_now = time_stamp.micros();
	while(time_stamp.micros() - time_now < delay_us);
}

//延时函数，单位为毫秒//Delay
void HAL_SYSTICK::ms(uint32_t delay_ms)
{
	while(delay_ms--)
		us(1000);
}



//中断服务函数要用C的方式进行编译
extern "C"
{
	void SysTick_Handler(void)
	{
		system_real_time_ms++;
	}
}






