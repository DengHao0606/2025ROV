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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "autocontrol.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  uint8_t state[6];
  PositionalPID x;
  PositionalPID y;
  PositionalPID z;
  PositionalPID rx;
  PositionalPID ry;
  PositionalPID rz;
} RobotController;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern CoordinateSystems robot;
extern RobotController robot_controller;
extern float servo0angle;
extern float openloop_thrust[6];

// extern int led_motion;
// extern int led_dataup;
// extern int led_uart4;
// extern int led_uart7;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GT_TX_Pin GPIO_PIN_14
#define GT_TX_GPIO_Port GPIOB
#define GT_RX_Pin GPIO_PIN_15
#define GT_RX_GPIO_Port GPIOB
#define LED0_H7_Pin GPIO_PIN_6
#define LED0_H7_GPIO_Port GPIOD
#define LED1_H7_Pin GPIO_PIN_7
#define LED1_H7_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
