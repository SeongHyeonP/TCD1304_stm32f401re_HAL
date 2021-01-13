/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
//#include "timer_conf.h"
//#include "ADC_conf.h"
//#include "UART_conf.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CCD_Output_Pin GPIO_PIN_0
#define CCD_Output_GPIO_Port GPIOC
#define ICG_Pin GPIO_PIN_0
#define ICG_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SH_Pin GPIO_PIN_3
#define SH_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* Data definitions */
#define CCDSize 3694
#define TxDataSize 7388
#define RxDataSize 12

/* CCD master clock in Hz */
/* The values presented here are the prescalable frequencies between
   1-2MHz with the STM32F401RE running at 84 MHz. Other frequencies
   are achievable when prescaling APB1 differently. 1 MHz does not
   seem to work well. */
//#define CCD_fm 1000000
//#define CCD_fm 1400000
//#define CCD_fm 1500000
//define CCD_fm 1680000
//#define CCD_fm 1750000
#define CCD_fm 2000000

/*  Comply with the CCD's timing requirements:
	The delay is dependent on the CCD_fM. These values appear to work:
		For 1.4-2.0 MHz use: ICG_delay = 11 and SH_delay = 12
		For 1.0 MHz use: ICG_delay = 12 and SH_delay = 12   */
#define SH_delay 12
#define ICG_delay 11
#define fm_delay 3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
