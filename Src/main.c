/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "fsm.h"
#include "wallet_states.h"
#include "coffe_states.h"
#include "temperature_states.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

osThreadId defaultTaskHandle;
osThreadId walletTaskHandle;
osThreadId temperatureTaskHandle;
osThreadId coffeTaskHandle;
osMutexId money_mutexHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int flags = 0;
static int money = 0;
static int coin_inserted = 0; //for debugging
int32_t temperature;
TickType_t coffe_delay_TimeInTicks = pdMS_TO_TICKS( 50 );
TickType_t coins_delay_TimeInTicks = pdMS_TO_TICKS( 100 );

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
void StartDefaultTask(void const * argument);
void StartWalletTask(void const * argument);
void StartTemperatureTask(void const * argument);
void StartCoffeTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static int button_coin_pressed(fsm_t* this){
  return flags & FLAG_BUTTON_COIN;
}

static int button_start_pressed (fsm_t* this) {
  int aux = (flags & FLAG_BUTTON_S);
  if(aux){
	if(xSemaphoreTake(money_mutexHandle,coffe_delay_TimeInTicks)== pdPASS){
		if(money>=COFFEE_PRICE){
			money=money-COFFEE_PRICE;
			xSemaphoreGive(money_mutexHandle);
			//printf("Enough money, Let´s Start!");
			return 1;
		}
		else{
			xSemaphoreGive(money_mutexHandle);
			//printf("Insert more money, please.");
			return 0;
		}
	}
  }
  return aux;
 }

static int button_cancell_pressed (fsm_t* this) {
  return (flags & FLAG_BUTTON_C); }

static int tim2_finished (fsm_t* this) {
  return (flags & FLAG_TIM2); }

static int tim3_finished (fsm_t* this) {
  return (flags & FLAG_TIM3); }

static void cup (fsm_t* this){
  flags =0;
  HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);
}

static void coffee (fsm_t* this)
{
  flags = 0;
  HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COFFE_LED_GPIO_Port, COFFE_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim3);

}

static void milk (fsm_t* this)
{
  flags = 0;
  HAL_GPIO_WritePin(COFFE_LED_GPIO_Port, COFFE_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MILK_LED_GPIO_Port, MILK_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim3);
}

static void finish (fsm_t* this)
{
  HAL_GPIO_WritePin(MILK_LED_GPIO_Port, MILK_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FINISH_LED_GPIO_Port, FINISH_LED_Pin, GPIO_PIN_SET);
  //int local_money;
  if(xSemaphoreTake(money_mutexHandle,coffe_delay_TimeInTicks)== pdPASS){
  		//local_money = money; //when i use printf
  		money=0;
  		xSemaphoreGive(money_mutexHandle);
  		flags = 0;
  		 //printf("Exchange: %d\n", local_money);
  	  }
}

static void add_money (fsm_t* this){
  //int local_money; //when i use printf
  if(xSemaphoreTake(money_mutexHandle,coins_delay_TimeInTicks)== pdPASS){
	  coin_inserted=10;
	  money+=coin_inserted;
	  //local_money = money; //when i use printf
	 xSemaphoreGive(money_mutexHandle);
	 flags = 0;
	 //printf("Coins inserted: %d\n", coin_inserted);
	 //printf("Total money: %d\n", local_money);
  }
}

static void return_money (fsm_t* this){
	//int local_money; //when i use printf
	if(xSemaphoreTake(money_mutexHandle,coins_delay_TimeInTicks)== pdPASS){
		//local_money = money; //when i use printf
		money=0;
		xSemaphoreGive(money_mutexHandle);
		flags = 0;
		 //printf("Total money: %d\n", local_money);
	  }

}

// Explicit COFFEE FSM description
static fsm_trans_t cofm[] = {
  { COFM_WAITING, button_start_pressed, COFM_CUP,     cup    },
  { COFM_CUP,     tim2_finished, COFM_COFFEE,  coffee },
  { COFM_COFFEE,  tim3_finished, COFM_MILK,    milk   },
  { COFM_MILK,    tim3_finished, COFM_WAITING, finish },
  {-1, NULL, -1, NULL },
};

// Explicit WALLET FSM description
static fsm_trans_t coinsm[] = {
  {WALLM_WAITING, button_coin_pressed, WALLM_WAITING, add_money},
  {WALLM_WAITING, button_cancell_pressed, WALLM_WAITING, return_money },
  {-1, NULL, -1, NULL },
};

/*--------------- Sensor temperature -----------------------------------*/
static int current_temperature () {
	if (__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC)){
		temperature = (((int32_t)HAL_ADC_GetValue(&hadc)*VDD_APPLI/VDD_CALIB)- (int32_t) *TEMP30_CAL_ADDR );
	  	temperature = temperature * (int32_t)(110 - 30);
	  	temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
	  	temperature = temperature + 30;
  }
	return temperature;
}

static int temp_high(fsm_t* this){
	int aux = flags & FLAG_TIM6;
	if(aux){
		int temp_aux = current_temperature();
		if(temp_aux>=TEMP_REF){
			return 1;
		}
		else{
			return 0;
		}
	}
	else{
		return aux;
	}
}

static int temp_low(fsm_t* this){
	int aux = flags & FLAG_TIM6;
	if(aux){
		int temp_aux = current_temperature();
		if(temp_aux<TEMP_REF){
			return 1;
		}
		else{
			return 0;
		}
	}
	else{
		return aux;
	}
}

static void led_on(fsm_t * this){
	flags = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}

static void led_off(fsm_t * this){
	flags = 0;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

static fsm_trans_t temperaturem[] = {
  {TEMP_WAITING, temp_high, TEMP_WAITING, led_on},
  {TEMP_WAITING, temp_low, TEMP_WAITING, led_off },
  {-1, NULL, -1, NULL },
};
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of money_mutex */
  osMutexDef(money_mutex);
  money_mutexHandle = osMutexCreate(osMutex(money_mutex));

  	fsm_t* coffem_fsm = fsm_new (cofm);
  	fsm_t* coinsm_fsm = fsm_new (coinsm);
  	fsm_t* temperature_fsm = fsm_new (temperaturem);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of walletTask */
  osThreadDef(walletTask, StartWalletTask, osPriorityNormal, 0, 128);
  walletTaskHandle = osThreadCreate(osThread(walletTask), coinsm_fsm);

  /* definition and creation of coffeTask */
    osThreadDef(coffeTask, StartCoffeTask, osPriorityBelowNormal, 0, 128);
    coffeTaskHandle = osThreadCreate(osThread(coffeTask), coffem_fsm);

  /* definition and creation of temperatureTask */
  osThreadDef(temperatureTask, StartTemperatureTask, osPriorityLow, 0, 128);
  temperatureTaskHandle = osThreadCreate(osThread(temperatureTask), temperature_fsm);


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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 3, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 60000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 48000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 449;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, START_LED_Pin|COFFE_LED_Pin|MILK_LED_Pin|FINISH_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COIN_BUTTON_Pin START_BUTTON_Pin CANCELL_BUTTON_Pin */
  GPIO_InitStruct.Pin = COIN_BUTTON_Pin|START_BUTTON_Pin|CANCELL_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : START_LED_Pin COFFE_LED_Pin MILK_LED_Pin FINISH_LED_Pin */
  GPIO_InitStruct.Pin = START_LED_Pin|COFFE_LED_Pin|MILK_LED_Pin|FINISH_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin){
		case COIN_BUTTON_Pin:
			flags |= FLAG_BUTTON_COIN;
			break;
		case START_BUTTON_Pin:
			flags |= FLAG_BUTTON_S;
			break;
		case CANCELL_BUTTON_Pin:
			flags |= FLAG_BUTTON_C;
			break;
		default:
			flags = 0;
			break;
	}

}
/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}
void StartWalletTask(void const * argument){
	fsm_t* pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = ( fsm_t * ) argument;
	xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
	fsm_fire(pcTaskName);
	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 150 ) );
	 }
}

void StartCoffeTask(void const * argument){
	fsm_t* pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = ( fsm_t * ) argument;
	xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
	fsm_fire(pcTaskName);
	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 300 ) );
	 }
}

void StartTemperatureTask(void const * argument){
	fsm_t* pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = ( fsm_t * ) argument;
	xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
	fsm_fire(pcTaskName);
	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
	 }
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
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
