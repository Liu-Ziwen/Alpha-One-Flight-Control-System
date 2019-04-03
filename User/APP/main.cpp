/**
  ******************************************************************************
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @brief   Main program body
  ******************************************************************************
**/	

#include "main.h"

uint8_t loop_timer_400hz = 0;
uint8_t loop_timer_200hz = 0;
uint8_t loop_timer_100hz = 0;
uint8_t loop_timer_50hz = 0;
uint8_t loop_timer_10hz = 0;
uint8_t loop_timer_5hz = 0;
uint8_t loop_timer_2hz = 0;
uint32_t timer_2p5ms = 0;

//////////////////////////////////////////////////////////////////////////////////////
//所有的dtime加起来应该不大于2500us
//否则会影响正常时序
uint32_t test_time_now;		/////
uint32_t test_time_last;	/////
uint16_t test_dtime_400hz;
uint16_t test_dtime_200hz;
uint16_t test_dtime_100hz;
uint16_t test_dtime_50hz;
uint16_t test_dtime_10hz;
uint16_t test_dtime_5hz;
uint16_t test_dtime_2hz;
//////////////////////////////////////////////////////////////////////////////////////


int main(void)
{
	/* HAL */
	hal_systick.init();
	hal_rgb.init();
	hal_spi.init(SPI_BaudRatePrescaler_4);
	hal_i2c.init();

	ms5611.init();
	mpu6k.init_with_check();
	hmc5883l.init_with_check();

	while(1)
	{
		if(time_stamp.micros() - timer_2p5ms >= 2500)
		{
			timer_2p5ms = time_stamp.micros();
			loop_timer_400hz++;
			loop_timer_200hz++;
			loop_timer_100hz++;
			loop_timer_50hz++;
			loop_timer_10hz++;
			loop_timer_5hz++;
			loop_timer_2hz++;
		}
		if(loop_timer_400hz>=1)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_400hz = 0;
			mpu6k.update();
			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_400hz = test_time_now - test_time_last;/////////////////////////
		}

		if(loop_timer_200hz>=2)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_200hz = 0;
			hmc5883l.update();		//I2C，读一次45us

			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_200hz = test_time_now - test_time_last;/////////////////////////
		}
		if(loop_timer_100hz>=4)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_100hz = 0;			
			ms5611.update();		//SPI读取气压计MS5611-01BA03

			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_100hz = test_time_now - test_time_last;/////////////////////////
		}
		if(loop_timer_50hz>=8)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_50hz = 0;
			
			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_50hz = test_time_now - test_time_last;/////////////////////////
		}
		if(loop_timer_10hz>=40)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_10hz = 0;
			
			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_10hz = test_time_now - test_time_last;/////////////////////////
		}
		if(loop_timer_5hz>=80)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_5hz = 0;
//			hal_rgb.update_color(9);
			
			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_5hz = test_time_now - test_time_last;/////////////////////////
		}
		if(loop_timer_2hz>=200)
		{
			test_time_last = time_stamp.micros();		///////////////////////////////
			
			loop_timer_2hz = 0;
			hal_rgb.update_color(9);
			
			test_time_now = time_stamp.micros();		///////////////////////////////
			test_dtime_2hz = test_time_now - test_time_last;/////////////////////////
		}
	}
}


