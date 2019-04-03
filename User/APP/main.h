/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    main.h
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H_
#define __MAIN_H_

/* Includes ------------------------------------------------------------------*/
#include <stm32f4xx.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Common Functions and Variables -----------*/
#include "com_math.h"
#include "com_global.h"

/* Hardware Abstract Layer ------------------*/
#include "hal_systick.h"
#include "hal_gpio_led.h"
//#include "hal_timer_pwm.h"
//#include "hal_exti_cap.h"
#include "hal_spi.h"
#include "hal_i2c.h"
//#include "hal_uart.h"
//#include "hal_dma.h"

/* Board Support Package --------------------*/
#include "bsp_ms5611.h"
#include "bsp_mpu6k.h"
#include "bsp_hmc5883l.h"
//#include "bsp_ublox.h"

/* Framework Module Libraries ---------------*/
//#include "fml_com_protocol.h"
//#include "fml_ano_protocol.h"

/* Applications -----------------------------*/


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
