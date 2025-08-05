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
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "memorymap.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "can_receive.h"
#include "HT_can_comm.h"
#include "HT_motor_control.h"
#include "INS_Task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
float gyro[3], accel[3], temp;
extern joint_motor_t joint_control;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t exit_flag=0;
uint8_t rising_falling_flag;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == sensor_Pin)
    {
        if(exit_flag == 0)
        {
         exit_flag = 1;
        rising_falling_flag = HAL_GPIO_ReadPin(sensor_GPIO_Port, sensor_Pin);
        }
			
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_UART5_Init();
  MX_SPI2_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART1_UART_Init();
  MX_FDCAN3_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	BSP_USART_Init();
//	BSP_FDCAN_Init();
	fdcan_filter_init();

HAL_GPIO_WritePin(POWER_OUT1_GPIO_Port,POWER_OUT1_Pin,1);
HAL_GPIO_WritePin(POWER_OUT2_GPIO_Port,POWER_OUT1_Pin,1);
//蜂鸣器
//	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
//	TIM12->CCR2 = 120;
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	/*****��������****/
//	HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��
//HAL_Delay(1000);
////�Ƴ�
////����

//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);//��
//HAL_Delay(5);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);
////�س�
//HAL_Delay(150);
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
////HAL_Delay(473);
//HAL_Delay(400);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��

//	/*****�����������****/
//	HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//��
//HAL_Delay(1000);
////�Ƴ�
////����

//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);//��
//HAL_Delay(30);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);
////�س�
//HAL_Delay(150);
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
////HAL_Delay(473);
//HAL_Delay(380);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//��

/****************************���ð汾***********************************************/
//	/***���ӽ�������****/
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��
//HAL_Delay(1000);
////�Ƴ�
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//����
//HAL_Delay(14);
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);//��


////�س�
//HAL_Delay(150);
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
////HAL_Delay(473);
//HAL_Delay(420);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,0);//��








/******���ԣ����ӽ�������*******/
//	//��ʼ��
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//��
//HAL_Delay(1000);
////�Ƴ�
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,1);//��
//HAL_Delay(50);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//����
////�س�
//HAL_Delay(150);
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,0);//����
////HAL_Delay(473);
//HAL_Delay(450);
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,1);//��
//

  while (1)
  {
//HAL_GPIO_WritePin(finger_GPIO_Port,finger_Pin,GPIO_PIN_SET);//��ס
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,GPIO_PIN_RESET);//����
//HAL_Delay(2000);		
//HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,GPIO_PIN_SET);//����
//		HAL_Delay(2000);
//		HAL_GPIO_WritePin(push_GPIO_Port,push_Pin,GPIO_PIN_RESET);
//		HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
