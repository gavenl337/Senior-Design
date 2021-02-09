/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Sensor1_Pin GPIO_PIN_0
#define Sensor1_GPIO_Port GPIOC
#define Sensor2_Pin GPIO_PIN_1
#define Sensor2_GPIO_Port GPIOC
#define Sensor3_Pin GPIO_PIN_2
#define Sensor3_GPIO_Port GPIOC
#define SIM_TX_Pin GPIO_PIN_2
#define SIM_TX_GPIO_Port GPIOA
#define SIM_RX_Pin GPIO_PIN_3
#define SIM_RX_GPIO_Port GPIOA
#define Pot1_Pin GPIO_PIN_6
#define Pot1_GPIO_Port GPIOA
#define Pot2_Pin GPIO_PIN_7
#define Pot2_GPIO_Port GPIOA
#define Pot3_Pin GPIO_PIN_4
#define Pot3_GPIO_Port GPIOC
#define Button_Pin GPIO_PIN_0
#define Button_GPIO_Port GPIOB
#define R_LED_Pin GPIO_PIN_1
#define R_LED_GPIO_Port GPIOB
#define G_LED_Pin GPIO_PIN_2
#define G_LED_GPIO_Port GPIOB
#define Heater_Pin GPIO_PIN_10
#define Heater_GPIO_Port GPIOB
#define CS3_Pin GPIO_PIN_6
#define CS3_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_7
#define CS2_GPIO_Port GPIOC
#define CS1_Pin GPIO_PIN_8
#define CS1_GPIO_Port GPIOC
#define Dip5_Pin GPIO_PIN_8
#define Dip5_GPIO_Port GPIOA
#define Dip4_Pin GPIO_PIN_9
#define Dip4_GPIO_Port GPIOA
#define Dip3_Pin GPIO_PIN_10
#define Dip3_GPIO_Port GPIOA
#define Dip2_Pin GPIO_PIN_11
#define Dip2_GPIO_Port GPIOA
#define Dip1_Pin GPIO_PIN_12
#define Dip1_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
