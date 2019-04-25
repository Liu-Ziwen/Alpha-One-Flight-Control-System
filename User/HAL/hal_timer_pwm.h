/**
  ******************************************************************************
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    12-April-2019
  * @brief   hal_timer_pwm.h
  ******************************************************************************
**/	

#ifndef __HAL_TIMER_PWM_H_
#define __HAL_TIMER_PWM_H_

//采用的电机顺序定义和APM中相同
#define MOTOR_1_PWM		TIM3->CCR4	//右前MOTOR_1
#define MOTOR_2_PWM		TIM3->CCR3	//左前MOTOR_2
#define MOTOR_3_PWM		TIM3->CCR2	//左后MOTOR_3
#define MOTOR_4_PWM		TIM3->CCR1	//右后MOTOR_4

#define MOTOR_1		1
#define MOTOR_2		2
#define MOTOR_3		3
#define MOTOR_4		4

////////////////////////////////////////////////////////////
#ifdef __cplusplus

class HAL_TIMER_PWM
{
public:
	void init(void);
	void set(uint8_t motor_n, uint32_t pwm);
private:
	void tim1_init(uint16_t frequency);
	void tim3_init(uint16_t frequency);
};

extern HAL_TIMER_PWM hal_pwm;







#endif

#endif







