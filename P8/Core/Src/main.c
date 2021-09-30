/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for led_verde */
osThreadId_t led_verdeHandle;
const osThreadAttr_t led_verde_attributes = {
  .name = "led_verde",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for led_amarillo */
osThreadId_t led_amarilloHandle;
const osThreadAttr_t led_amarillo_attributes = {
  .name = "led_amarillo",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for control_leds */
osThreadId_t control_ledsHandle;
const osThreadAttr_t control_leds_attributes = {
  .name = "control_leds",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for respuesta */
osThreadId_t respuestaHandle;
const osThreadAttr_t respuesta_attributes = {
  .name = "respuesta",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for num_pulsaciones */
osMessageQueueId_t num_pulsacionesHandle;
const osMessageQueueAttr_t num_pulsaciones_attributes = {
  .name = "num_pulsaciones"
};
/* Definitions for semaverde */
osSemaphoreId_t semaverdeHandle;
const osSemaphoreAttr_t semaverde_attributes = {
  .name = "semaverde"
};
/* Definitions for semaamarillo */
osSemaphoreId_t semaamarilloHandle;
const osSemaphoreAttr_t semaamarillo_attributes = {
  .name = "semaamarillo"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void Start_led_verde(void *argument);
void Start_led_amarillo(void *argument);
void Start_control_leds(void *argument);
void Start_respuesta(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of semaverde */
  semaverdeHandle = osSemaphoreNew(5, 5, &semaverde_attributes);

  /* creation of semaamarillo */
  semaamarilloHandle = osSemaphoreNew(5, 5, &semaamarillo_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of num_pulsaciones */
  num_pulsacionesHandle = osMessageQueueNew (3, sizeof(uint16_t), &num_pulsaciones_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of led_verde */
  led_verdeHandle = osThreadNew(Start_led_verde, NULL, &led_verde_attributes);

  /* creation of led_amarillo */
  led_amarilloHandle = osThreadNew(Start_led_amarillo, NULL, &led_amarillo_attributes);

  /* creation of control_leds */
  control_ledsHandle = osThreadNew(Start_control_leds, NULL, &control_leds_attributes);

  /* creation of respuesta */
  respuestaHandle = osThreadNew(Start_respuesta, NULL, &respuesta_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, verde_Pin|amarillo_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : tamper_Pin */
  GPIO_InitStruct.Pin = tamper_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(tamper_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : wkup_Pin */
  GPIO_InitStruct.Pin = wkup_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(wkup_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : verde_Pin amarillo_Pin */
  GPIO_InitStruct.Pin = verde_Pin|amarillo_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_Start_led_verde */
/**
* @brief Function implementing the led_verde thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_led_verde */
void Start_led_verde(void *argument)
{
  /* USER CODE BEGIN Start_led_verde */
  /* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire (semaverdeHandle,0xFFFFFFFF);
		HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_SET);
		osDelay(200);
		HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_RESET);
		osDelay(200);
	}
  /* USER CODE END Start_led_verde */
}

/* USER CODE BEGIN Header_Start_led_amarillo */
/**
* @brief Function implementing the led_amarillo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_led_amarillo */
void Start_led_amarillo(void *argument)
{
  /* USER CODE BEGIN Start_led_amarillo */
	/* Infinite loop */
	for(;;)
	{
		osSemaphoreAcquire (semaamarilloHandle,0xFFFFFFFF);
		HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_SET);
		osDelay(200);
		HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_RESET);
		osDelay(200);
	}
  /* USER CODE END Start_led_amarillo */
}

/* USER CODE BEGIN Header_Start_control_leds */
/**
* @brief Function implementing the control_leds thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_control_leds */
//void Start_control_leds(void *argument)
//{
//	/* USER CODE BEGIN Start_control_leds */
//	/* Infinite loop */
//	unsigned short respuesta; //enteros de 16 bits sin signo
//	while (HAL_GPIO_ReadPin(GPIOC, tamper_Pin) == 0)
//		osDelay(100);
//	osMessageQueueReset (num_pulsacionesHandle);
//	for (;;) {
//		osMessageQueueGet(num_pulsacionesHandle, &respuesta, 0, 0xFFFFFFFF);
//		if (respuesta % 2 == 0) {
//			HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_SET);
//		} else {
//			HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_SET);
//		}
//		osDelay(2000);
//		HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_RESET);
//		osDelay(10); //dejar esta línea fuera del if en la fase 4
//	}
//	/* USER CODE END Start_control_leds */
//}

void Start_control_leds(void *argument)
{
	/* USER CODE BEGIN Start_control_leds */
	/* Infinite loop */
	unsigned short i, numInterVerde, numInterAmarillo, respuesta;
	while(HAL_GPIO_ReadPin(GPIOC, tamper_Pin)==0)
		osDelay(50);
	for (;;) {
		if (HAL_GPIO_ReadPin(GPIOC, tamper_Pin) == 0) {
			numInterVerde = osKernelGetTickCount();
			numInterAmarillo = numInterVerde%5;
			numInterVerde = (numInterVerde/100)%5;
			numInterAmarillo++;
			numInterVerde++;
			for (i = 0; i < numInterVerde; i++)
				osSemaphoreRelease(semaverdeHandle);
			for (i = 0; i < numInterAmarillo; i++)
				osSemaphoreRelease(semaamarilloHandle);
			osMessageQueueGet(num_pulsacionesHandle, &respuesta, 0, 0xFFFFFFFF);
			if(respuesta == numInterVerde+numInterAmarillo){
				HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_SET);
				osDelay(1000);
			}else{
				HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_SET);
				osDelay(1000);
			}
			osDelay(1000);
			HAL_GPIO_WritePin(GPIOC, amarillo_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, verde_Pin, GPIO_PIN_RESET);
		}
		osDelay(10);
	}
	/* USER CODE END Start_control_leds */
}

/* USER CODE BEGIN Header_Start_respuesta */
/**
* @brief Function implementing the respuesta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_respuesta */
void Start_respuesta(void *argument)
{
	/* USER CODE BEGIN Start_respuesta */
	/* Infinite loop */
	unsigned short conta;
	for(;;)
	{
		if (HAL_GPIO_ReadPin(GPIOA, wkup_Pin) == 1) {
			conta++;
			while (HAL_GPIO_ReadPin(GPIOA, wkup_Pin) == 1)
				osDelay(10);
		}
		if (HAL_GPIO_ReadPin(GPIOC, tamper_Pin) == 0 && conta!=0) {
			osMessageQueuePut(num_pulsacionesHandle,&conta,0,0);
			conta = 0;
		}
		osDelay(10);
	}
	/* USER CODE END Start_respuesta */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
