/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "comm.h"
#include "thrust.h"
#include "usart.h"
#include "filter.h"

#include "autocontrol.h"

#include "cJSON.h"
#include "json_process.h"
#include "can_process.h"
#include "RS485_process.h"
#include "wt_usart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define thrust_mean_filter_length 20
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
RobotController robot_controller;
float           openloop_thrust[6] = {0}; // 0~5 correspond x y z rx ry rz

MeanFilter meanfilter[6];
 
int led_motion   = 0;
int led_dataup   = 0;
int led_uart4    = 0;
int led_uart7    = 0;
int led_main     = 0;
int led_watchdog = 0;
int led_ms5837   = 0;

int threadmonitor_tim2  = 30;
int threadmonitor_tim3  = 30;
int threadmonitor_uart1 = 300;
int threadmonitor_uart7 = 300;
int threadmonitor_uart8 = 300;
int16_t servo0angletest;
int start = 0;

uint8_t transbuf[157] = {0};

// float measureddepth = 0;
// float realdepth     = 0;
// float startdepth    = 0;
// float checkeddepth  = 0;
float received_depth = 0;
float received_temp = 0;
uint8_t can_rx_data[8];
uint8_t angle[8] = {0x50, 0x03, 0x00, 0x3D, 0x00, 0x03, 0x99, 0x86};
uint8_t accelarate[8] = {0x50, 0x03, 0x00, 0x34, 0x00, 0x03, 0x49, 0x84};
// extern float servo0angle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
// int fputc(int ch,FILE *f)
// {
//   uint8_t temp[1]={ch};
//   HAL_UART_Transmit(&huart7,temp,1,2);
//   return ch;
// }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
 * 函数�??????????????????????: HAL_TIM_PeriodElapsedCallback
 * 描述  : 定时器中断处�??????????????????????
 * 输入  : TIM_HandleTypeDef *htim 定时器地�??????????????????????
 * 输出  : /
 * 备注  : 用于处理数据
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static float motorthrust[6]        = {0}; // 0~5 correspond motor0~5
    static float askedthrust[6]        = {0};
    static float motorthrust_filted[6] = {0};


    //  1号定时器中断
    //  频率 20hz
    if (htim == (&htim1))
    { // watch dog
        threadmonitor_tim2--;
        threadmonitor_tim3--;
        threadmonitor_uart1--;
        threadmonitor_uart7--;
        threadmonitor_uart8--;

        if (threadmonitor_uart1 <= 0)
        {
            HAL_UART_Receive_IT(&huart1, uart1rec.buf, 1);
            __HAL_UART_CLEAR_OREFLAG(&huart1);
            threadmonitor_uart1 = 30;
        }
        if (threadmonitor_tim2 <= 0)
        {
            __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim2);
            threadmonitor_tim2 = 30;
        }
        if (threadmonitor_tim3 <= 0)
        {
            __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
            HAL_TIM_Base_Start_IT(&htim3);
            threadmonitor_tim3 = 30;
        }
        if (threadmonitor_uart7 <= 0)
        {
            HAL_UART_Receive_IT(&huart7, uart7rec.buf, 1);
            __HAL_UART_CLEAR_OREFLAG(&huart7);
            threadmonitor_uart7 = 30;
        }
        if (threadmonitor_uart8 <= 0)
        {
            HAL_UART_Receive_IT(&huart8, uart8rec.buf, 1);
            __HAL_UART_CLEAR_OREFLAG(&huart8);
            threadmonitor_uart8 = 30;
        }
        // led
        if (led_watchdog)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);
        led_watchdog = !led_watchdog;
    }
    //  2号定时器中断
    //  频率 16hz,数据上行
    else if (htim == (&htim2))
    {
        threadmonitor_tim3 = 20;
        send_depth_temperature();
    }
    //  3号定时器中断
    //  频率 30hz
    else if (htim == (&htim3))
    {
        threadmonitor_tim2 = 20;
        // allocate thrust
        ThrustAllocate(openloop_thrust, motorthrust);

        // thrust filter
        // for (int i = 0; i < 6; i++) { motorthrust_filted[i] = meanfilter[i].refresh(&(meanfilter[i]), motorthrust[i]); }
        for (int i = 0; i < 6; i++) { motorthrust_filted[i] = motorthrust[i]; }

        // Convert thrust signal to PWM signal
        MotorPwmRefresh(motorthrust_filted);

        // led
        if (led_motion)
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
        led_motion = !led_motion;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_FDCAN1_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_FDCAN2_Init();
  /* USER CODE BEGIN 2 */

  // uart it start
  CommInit();
  FDCAN1_Config();
  FDCAN2_Config();

  // motor init
  MotorInit();
  HAL_Delay(50);

  transbuf[0]   = 0xfa;
  transbuf[1]   = 0xaf;
  transbuf[2]   = 0x00;
  transbuf[155] = 0xfb;
  transbuf[156] = 0xbf;

  MeanFilter_Init(&(meanfilter[0]), thrust_mean_filter_length);
  MeanFilter_Init(&(meanfilter[1]), thrust_mean_filter_length);
  MeanFilter_Init(&(meanfilter[2]), thrust_mean_filter_length);
  MeanFilter_Init(&(meanfilter[3]), thrust_mean_filter_length);
  MeanFilter_Init(&(meanfilter[4]), thrust_mean_filter_length);
  MeanFilter_Init(&(meanfilter[5]), thrust_mean_filter_length);

  ThrustCurveInit(&(thrustcurve[0]));
  ThrustCurveInit(&(thrustcurve[1]));
  ThrustCurveInit(&(thrustcurve[2]));
  ThrustCurveInit(&(thrustcurve[3]));
  ThrustCurveInit(&(thrustcurve[4]));
  ThrustCurveInit(&(thrustcurve[5]));
  // start data process
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

  // 设置电机2的位置为45度，优先�??1
  // set_position(&hfdcan2, 1, 0, 0xFF);
  // HAL_Delay(5000);
  // set_position(&hfdcan2, 1, 90, 0xFF);
  // HAL_Delay(5000);
  // emergency_stop(&hfdcan2, 1, 0xFF);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    
    // HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8|RCC_PERIPHCLK_FDCAN
                              |RCC_PERIPHCLK_UART7;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 32;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 2;
  PeriphClkInitStruct.PLL3.PLL3N = 60;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 8;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
