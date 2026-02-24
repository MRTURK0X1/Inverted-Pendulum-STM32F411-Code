/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "message_buffer.h"
#include "utility.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern TaskHandle_t control_task_handle;
extern TaskHandle_t command_task_handle;
extern TaskHandle_t uartTx_task_handle;

extern MessageBufferHandle_t state_buffer;
extern MessageBufferHandle_t debug_buffer;

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
void start_sampler();
void start_cartEncoder();
void start_pendulumEncoder();


// TASK HANDLERS
void control_task_handler(void *argument);
void command_task_handler(void *argument);
void uartTx_task_handler(void *argument);
void uartRx_task_handler(void *argument);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USR_BUTTON_Pin GPIO_PIN_0
#define USR_BUTTON_GPIO_Port GPIOA
#define BTS_ENABLE_Pin GPIO_PIN_0
#define BTS_ENABLE_GPIO_Port GPIOB
#define OUTPUT_ENABLE_Pin GPIO_PIN_11
#define OUTPUT_ENABLE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define USR_BUTTON_Pin GPIO_PIN_0
#define USR_BUTTON_GPIO_Port GPIOA
#define BTS_ENABLE_Pin GPIO_PIN_0
#define BTS_ENABLE_GPIO_Port GPIOB
#define OUTPUT_ENABLE_Pin GPIO_PIN_11
#define OUTPUT_ENABLE_GPIO_Port GPIOA
#define SIMULATION_MODE   		0
#define DEBUG_MODE				1




/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
