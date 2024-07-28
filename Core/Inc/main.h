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
#define LEFT_MOTORS_DIR_PIN_Pin GPIO_PIN_2
#define LEFT_MOTORS_DIR_PIN_GPIO_Port GPIOC
#define RIGHT_MOTORS_DIR_PIN_Pin GPIO_PIN_3
#define RIGHT_MOTORS_DIR_PIN_GPIO_Port GPIOC
#define ALL_MOTORS_ENABLE_PIN_Pin GPIO_PIN_0
#define ALL_MOTORS_ENABLE_PIN_GPIO_Port GPIOA
#define ALL_MOTORS_BRAKE_PIN_Pin GPIO_PIN_1
#define ALL_MOTORS_BRAKE_PIN_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MOTOR4_W_Pin GPIO_PIN_13
#define MOTOR4_W_GPIO_Port GPIOB
#define MOTOR4_V_Pin GPIO_PIN_14
#define MOTOR4_V_GPIO_Port GPIOB
#define MOTOR4_U_Pin GPIO_PIN_15
#define MOTOR4_U_GPIO_Port GPIOB
#define MOTOR3_W_Pin GPIO_PIN_6
#define MOTOR3_W_GPIO_Port GPIOC
#define MOTOR3_V_Pin GPIO_PIN_7
#define MOTOR3_V_GPIO_Port GPIOC
#define MOTOR3_U_Pin GPIO_PIN_8
#define MOTOR3_U_GPIO_Port GPIOC
#define MOTOR2_W_Pin GPIO_PIN_9
#define MOTOR2_W_GPIO_Port GPIOC
#define MOTOR2_V_Pin GPIO_PIN_8
#define MOTOR2_V_GPIO_Port GPIOA
#define MOTOR2_U_Pin GPIO_PIN_9
#define MOTOR2_U_GPIO_Port GPIOA
#define MOTOR1_W_Pin GPIO_PIN_10
#define MOTOR1_W_GPIO_Port GPIOA
#define MOTOR1_V_Pin GPIO_PIN_11
#define MOTOR1_V_GPIO_Port GPIOA
#define MOTOR1_U_Pin GPIO_PIN_12
#define MOTOR1_U_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */


#define DEBUG_MODE 1


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
