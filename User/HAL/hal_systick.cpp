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

//�����ʱ����ʱ���//TimeStamp
uint32_t HAL_SYSTICK::millis(void)
{
	return system_real_time_ms;
}

//΢���ʱ����ʱ���//TimeStamp
uint32_t HAL_SYSTICK::micros(void)
{
	do
	{
		ms_cnt = system_real_time_ms;
		systick_cycle = SysTick->VAL;
	}while(ms_cnt != system_real_time_ms);
	//��������Ѽ�ʱ��΢��Ϊ�ܵ�΢����
	return (ms_cnt * 1000) + (usticks * 1000 - systick_cycle) / usticks;
}

//��ʱ��������λΪ΢��//Delay
void HAL_SYSTICK::us(uint32_t delay_us)
{
	time_now = time_stamp.micros();
	while(time_stamp.micros() - time_now < delay_us);
}

//��ʱ��������λΪ����//Delay
void HAL_SYSTICK::ms(uint32_t delay_ms)
{
	while(delay_ms--)
		us(1000);
}



//�жϷ�����Ҫ��C�ķ�ʽ���б���
extern "C"
{
	void SysTick_Handler(void)
	{
		system_real_time_ms++;
	}
}






