/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
enum HeaterState
{
	OFF,
	PRE_HEAT,
	FULL_HEAT
};

enum ActiveHeaterBank
{
	HEATER_BANK_0,
	HEATER_BANK_1,
	HEATER_BANK_2,
	HEATER_BANK_3
};
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
#define Blue_Button_Interrupt_Pin GPIO_PIN_13
#define Blue_Button_Interrupt_GPIO_Port GPIOC
#define Blue_Button_Interrupt_EXTI_IRQn EXTI15_10_IRQn
#define IR_Input_Interrupt_Pin GPIO_PIN_7
#define IR_Input_Interrupt_GPIO_Port GPIOA
#define IR_Input_Interrupt_EXTI_IRQn EXTI9_5_IRQn
#define HEATER_BANK_3_Pin GPIO_PIN_3
#define HEATER_BANK_3_GPIO_Port GPIOB
#define HEATER_BANK_0_Pin GPIO_PIN_10
#define HEATER_BANK_0_GPIO_Port GPIOB
#define HEATER_BANK_1_Pin GPIO_PIN_4
#define HEATER_BANK_1_GPIO_Port GPIOB
#define HEATER_BANK_2_Pin GPIO_PIN_5
#define HEATER_BANK_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define FULL_HEAT_STOPPOINT 150
#define PRE_HEAT_SETPOINT 125
#define PRE_HEAT_DEADBAND 0
#define THERMISTOR_RESISTOR 5.00
#define HEATER_COUNT 8
#define HEATER_BANK_COUNT 4
#define COMMAND_BUFFER_SIZE 25
#define COMMAND_QUEUE_SIZE 10
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
