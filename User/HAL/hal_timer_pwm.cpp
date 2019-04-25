/**
  ******************************************************************************
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    12-April-2019
  * @brief   hal_timer_pwm.cpp
  ******************************************************************************
**/	

#include "main.h"

HAL_TIMER_PWM hal_pwm;

void HAL_TIMER_PWM::init(void)
{
	tim1_init(50);
	tim3_init(50);
	set(MOTOR_1,1040U);
	set(MOTOR_2,1040U);
	set(MOTOR_3,1040U);
	set(MOTOR_4,1040U);
}

void HAL_TIMER_PWM::set(uint8_t motor_n, uint32_t pwm)
{
	switch(motor_n)
	{
		case MOTOR_1:
			MOTOR_1_PWM = pwm;
		break;
		
		case MOTOR_2:
			MOTOR_2_PWM = pwm;
		break;
		
		case MOTOR_3:
			MOTOR_3_PWM = pwm;
		break;
		
		case MOTOR_4:
			MOTOR_4_PWM = pwm;
		break;
	}
}

void HAL_TIMER_PWM::tim1_init(uint16_t frequency)
{
	uint16_t Period;
	Period=1000000/frequency;

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	//GPIO复用设置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);
	//GPIO配置：PA6~9，PB0~1			
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//TIM1配置
	TIM_TimeBaseStructure.TIM_Period = Period-1;		      	//自动重装值TIM_Period+1(1000,1ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 168-1;	              	//预分频  168M/(TIM_Prescaler+1)==1M,计时一次为1us
	//预分频  84M/(TIM_Prescaler+1)==1M,计时一次为1us
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 	//向上计数
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); 		//初始化TIMx
	//PWM配置：模式PWM1(当 CNT<CCRx 的时候有效)，有效位设置为高位 TIM_OCPolarity_High
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 		//选择定时器模式
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 	//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 	//输出极性
	//PWM通道配置
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);  										
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  										
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  
	//
	TIM_ARRPreloadConfig(TIM1,ENABLE);						//ARPE使能
	TIM_Cmd(TIM1, ENABLE);	                      //使能TIM1		
	//这一句对于TIM1是必要的，TIM3用不到
	TIM_CtrlPWMOutputs(TIM1,ENABLE);						//高级定时器TIM1和TIM8需要配置刹车和死区寄存器BDTR
}

void HAL_TIMER_PWM::tim3_init(uint16_t frequency)
{
	;
}




