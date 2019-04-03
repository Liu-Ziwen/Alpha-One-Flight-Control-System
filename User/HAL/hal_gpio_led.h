/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_gpio_led.h
  */

#ifndef __HAL_GPIO_LED_H_
#define __HAL_GPIO_LED_H_

////////////////////////////////////////////////////////////

//LED1_BLUE
#define LED_BLUE				1
#define GPIO_PIN_LED_BLUE		GPIO_Pin_12
#define GPIO_LED_BLUE			GPIOB
#define RCC_GPIO_LED_BLUE		RCC_AHB1Periph_GPIOB

//LED2_GREEN
#define LED_GREEN				2
#define GPIO_PIN_LED_GREEN		GPIO_Pin_13
#define GPIO_LED_GREEN			GPIOB
#define RCC_GPIO_LED_GREEN		RCC_AHB1Periph_GPIOB

//LED3_RED
#define LED_RED					3
#define GPIO_PIN_LED_RED		GPIO_Pin_13
#define GPIO_LED_RED			GPIOC
#define RCC_GPIO_LED_RED		RCC_AHB1Periph_GPIOC

//COLOR_DEFINE
#define RGBOFF		0
#define BLUE		1
#define GREEN		2
#define INDIGO		3
#define RED			4
#define PURPLE		5
#define YELLOW		6
#define WHITE		7

////////////////////////////////////////////////////////////
#ifdef __cplusplus

class HAL_GPIO_LED
{
public:
	void init(void);
	void update_color(uint8_t mode);	
private:
	void off(uint8_t number);
	void on(uint8_t number);
	void toggle(uint8_t number);
	void control(uint8_t color);
};
extern HAL_GPIO_LED hal_rgb;

#endif
#endif









