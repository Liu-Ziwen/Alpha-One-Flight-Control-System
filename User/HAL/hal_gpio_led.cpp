/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_gpio_led.cpp
  * @brief	 LED1_BLUE//LED2_GREEN//LED3_RED
  */

#include "main.h"
HAL_GPIO_LED hal_rgb;

void HAL_GPIO_LED::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_GPIO_LED_BLUE|RCC_GPIO_LED_GREEN|RCC_GPIO_LED_RED,ENABLE);//使能时钟

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
	
	//LED端口初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED_BLUE;
	GPIO_Init(GPIO_LED_BLUE,&GPIO_InitStructure);

	//LED_RED端口初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED_GREEN;
	GPIO_Init(GPIO_LED_GREEN,&GPIO_InitStructure);

	//LED_GREEN端口初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_PIN_LED_RED;  
	GPIO_Init(GPIO_LED_RED,&GPIO_InitStructure);
	
	off(LED_BLUE);
	off(LED_GREEN);
	off(LED_RED);
	
	on(LED_BLUE);
	on(LED_GREEN);
	on(LED_RED);
}

void HAL_GPIO_LED::off(uint8_t number)
{
	switch(number) 
	{
		case 1:
			GPIO_SetBits(GPIO_LED_BLUE,GPIO_PIN_LED_BLUE);
		  break;
		case 2:
			GPIO_SetBits(GPIO_LED_GREEN,GPIO_PIN_LED_GREEN);
		  break;		
		case 3:
			GPIO_SetBits(GPIO_LED_RED,GPIO_PIN_LED_RED);
		  break;	
	  default:
			break;
	}
}

void HAL_GPIO_LED::on(uint8_t number)
{
	switch(number) 
	{
		case 1:
			GPIO_ResetBits(GPIO_LED_BLUE,GPIO_PIN_LED_BLUE);
		  break;
		case 2:
			GPIO_ResetBits(GPIO_LED_GREEN,GPIO_PIN_LED_GREEN);
		  break;		
		case 3:
			GPIO_ResetBits(GPIO_LED_RED,GPIO_PIN_LED_RED);
		  break;	
	  default:
			break;
	}
}

void HAL_GPIO_LED::toggle(uint8_t number)
{
	switch(number) 
	{
		case 1:
			GPIO_ToggleBits(GPIO_LED_BLUE,GPIO_PIN_LED_BLUE);
		  break;
		case 2:
			GPIO_ToggleBits(GPIO_LED_GREEN,GPIO_PIN_LED_GREEN);
		  break;		
		case 3:
			GPIO_ToggleBits(GPIO_LED_RED,GPIO_PIN_LED_RED);
		  break;	
	  default:
			break;
	}
}

void HAL_GPIO_LED::control(uint8_t color)
{
	static uint8_t color_buff[3]={0};
	color_buff[0]=(color&1)>>0;
	color_buff[1]=(color&2)>>1;
	color_buff[2]=(color&4)>>2;
	
	if(color_buff[0])
	{
		GPIO_ToggleBits(GPIO_LED_BLUE,GPIO_PIN_LED_BLUE);
	}
	else GPIO_SetBits(GPIO_LED_BLUE,GPIO_PIN_LED_BLUE);
	
	if(color_buff[1])
	{
		GPIO_ToggleBits(GPIO_LED_GREEN,GPIO_PIN_LED_GREEN);
	}
	else GPIO_SetBits(GPIO_LED_GREEN,GPIO_PIN_LED_GREEN);
	
	if(color_buff[2])
	{
		GPIO_ToggleBits(GPIO_LED_RED,GPIO_PIN_LED_RED);
	}
	else GPIO_SetBits(GPIO_LED_RED,GPIO_PIN_LED_RED);
}

void HAL_GPIO_LED::update_color(uint8_t mode)
{
	static uint8_t status;
	status++;
	if(status>=10)status=0;
	
	switch(mode)
	{
		case 1:	//红绿黄连续闪烁
			switch(status)
			{
				case 1:case 2:
					control(RED);break;
				case 3:case 4:
					control(GREEN);break;
				case 5:case 6:
					control(YELLOW);break;
				case 7:
					control(RGBOFF);break;
			}break;
			
		case 2:	//黄灯四闪
			switch(status)
			{
				case 1:case 2:case 3:case 4:case 5:case 6:case 7:case 8:
					control(YELLOW);break;
			}break;
			
		case 3:	//绿灯慢闪
			switch(status)
			{
				case 1:case 6:
					control(GREEN);break;
			}break;
			
		case 4:	//黄灯慢闪
			switch(status)
			{
				case 1:case 6:
					control(YELLOW);break;
			}break;
			
		case 5:	//绿灯双闪
			switch(status)
			{
				case 1:case 2:case 3:case 4:case 6:case 7:case 8:case 9:
					control(GREEN);break;
			}break;	
			
		case 6:	//紫灯双闪
			switch(status)
			{
				case 1:case 2:case 3:case 4:case 6:case 7:case 8:case 9:
					control(PURPLE);break;
			}break;
			
		case 7:	//蓝灯快闪
			switch(status)
			{
				case 0:case 1:case 2:case 3:case 4:case 5:case 6:case 7:case 8:case 9:
					control(BLUE);break;
			}break;		
		case 8:	//绿灯快闪
			switch(status)
			{
				case 0:case 1:case 2:case 3:case 4:case 5:case 6:case 7:case 8:case 9:
					control(GREEN);break;
			}break;		
		case 9:	//绿灯闪烁//智能飞行模式（与其他飞行模式交替闪烁）
			switch(status)
			{
				case 0:case 1:case 2:case 3:case 4:case 5:case 6:case 7:case 8:case 9:
					control(GREEN);break;
			}break;		
		case 10://红灯闪烁//陀螺仪零偏校准不成功
			switch(status)
			{
				case 0:case 1:case 2:case 3:case 4:case 5:case 6:case 7:case 8:case 9:
					control(RED);break;
			}break;			
	}
}

