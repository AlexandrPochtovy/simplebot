/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32f1xx_ll_i2c.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define requestOn(req, mask) req |= mask
#define requestOff(req, mask) req &= ~mask
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define WheelLeft_Pin LL_GPIO_PIN_0
#define WheelLeft_GPIO_Port GPIOA
#define WheelRight_Pin LL_GPIO_PIN_1
#define WheelRight_GPIO_Port GPIOA
#define AIN2_Pin LL_GPIO_PIN_12
#define AIN2_GPIO_Port GPIOB
#define AIN1_Pin LL_GPIO_PIN_13
#define AIN1_GPIO_Port GPIOB
#define BIN1_Pin LL_GPIO_PIN_14
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin LL_GPIO_PIN_15
#define BIN2_GPIO_Port GPIOB
#define Bamper_Pin LL_GPIO_PIN_12
#define Bamper_GPIO_Port GPIOA
#define Bamper_EXTI_IRQn EXTI15_10_IRQn
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
#define LIMIT 				100  	// предельное раccто�?ние до преп�?т�?тви�?, мм
#define USART_POOL_PERIOD 	50		//usart period check data, msec
#define LASER_POOL_PERIOD 	15		//laser range sensor read data, msec
#define IMU_POOL_PERIOD			20		//imu sensors (accel, gyro mag) read data, msec
#define PACH_CALC_PERIOD		5			//trajectory calculate period
#define PID_CALC_PERIOD			10		//PID calculate period
#define SENSOR_POOL_PERIOD	500		//other sensor read data, msec

#define COM_REQUES_MASK			0x00000001U	//0 bit
#define	LIDAR_REQUEST_MASK	0x00000002U	//1 bit
#define	RES_REQUEST_MASK		0x00000004U	//2 bit
#define PACH_CALC_MASK			0x00000008U	//3 bit
#define PID_CALC_MASK				0x00000100U	//8 bit

#define RPS_KOEFFICIENT 208.0f
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
