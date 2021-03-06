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
#include "coffee_states.h"
#include "wallet_states.h"
#include "temperature_states.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
osThreadId coffeeTaskHandle;
osThreadId walletTaskHandle;
osThreadId temperatureTaskHandle;
osMutexId moneyHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int flag_sensor = 0;
int32_t temperature;
int flag_coin_button = 0;
int money = 0;
int flag_start_button = 0;
int flag_cancell_button = 0;
int flag_tim2=0;
int flag_tim3=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void const * argument);
void StartCoffeeTask(void const * argument);
void StartWalletTask(void const * argument);
void StartSensorTask(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static int newCoin(int coin){
	money+=coin;
	return money;
}

static int noCoffee(){
	money=0;
	return money;
}

static int exchange(){
	money=money- COFFEE_PRICE;
	return money;
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/*---------- Wallet FSM -----------------------------------------------*/


static int button_coin_pressed(fsm_t* this){
	if(flag_coin_button){
		flag_coin_button =0;
		return 1;
		}
	else{
		return 0;}
}

static void add_money (fsm_t* this){
  //int local_money; //when i use printf
  if(xSemaphoreTake(moneyHandle,( TickType_t ) 10)== pdTRUE){
	  //local_money=newCoin();
	  newCoin(60);
	 xSemaphoreGive(moneyHandle);
	 //printf("Total money: %d\n", local_money);
  }
}

static int button_cancell_pressed (fsm_t* this) {
	if(flag_cancell_button){
		flag_cancell_button=0;
		return 1;
		}
	else{
		return 0;}
}

static void return_money (fsm_t* this){
	//int local_money;
	if(xSemaphoreTake(moneyHandle,( TickType_t ) 10)== pdTRUE){
		//local_money = newCoin(0);
		noCoffee();
		xSemaphoreGive(moneyHandle);
		 //printf("Your money: %d\n", local_money);
	  }
}

static fsm_trans_t coinsm[] = {
  {WALLM_WAITING, button_coin_pressed, WALLM_WAITING, add_money},
  {WALLM_WAITING, button_cancell_pressed, WALLM_WAITING, return_money },
  {-1, NULL, -1, NULL },
};

/*---------- Coffee FSM -----------------------------------------------*/

static int button_start_pressed (fsm_t* this) {
 int result=0;
  if(flag_start_button){
	  if(xSemaphoreTake(moneyHandle,( TickType_t ) 10)== pdTRUE){
		if(money>=COFFEE_PRICE){
			exchange();
			result = 1;
			xSemaphoreGive(moneyHandle);
			flag_start_button =0;
			//printf("Enough money, Let�s Start!");
		}
		else{
			xSemaphoreGive(moneyHandle);
			flag_start_button =0;
		}

	}
  }
  return result;

 }

static int tim2_finished (fsm_t* this) {
	if(flag_tim2){
		HAL_TIM_Base_Stop_IT(&htim2);
		flag_tim2 =0;
		return 1;
	}
	else{
		return 0;}
	}


static void cup (fsm_t* this){
  HAL_GPIO_WritePin(FINISH_LED_GPIO_Port, FINISH_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);
}

static void coffee (fsm_t* this)
{
  HAL_GPIO_WritePin(START_LED_GPIO_Port, START_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(COFFEE_LED_GPIO_Port, COFFEE_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);

}

static void milk (fsm_t* this)
{
  HAL_GPIO_WritePin(COFFEE_LED_GPIO_Port, COFFEE_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MILK_LED_GPIO_Port, MILK_LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);
}

static void finish (fsm_t* this)
{
  HAL_GPIO_WritePin(MILK_LED_GPIO_Port, MILK_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FINISH_LED_GPIO_Port, FINISH_LED_Pin, GPIO_PIN_SET);
  //int local_money;
  if(xSemaphoreTake(moneyHandle,( TickType_t ) 10)== pdTRUE){
  		//local_money = newCoin(0); //when i use printf
  		money=0;
  		xSemaphoreGive(moneyHandle);
  		 //printf("Exchange: %d\n", local_money);
  	  }
}


// Explicit COFFEE FSM description
static fsm_trans_t cofm[] = {
  { COFM_WAITING, button_start_pressed, COFM_CUP,     cup    },
  { COFM_CUP,     tim2_finished, COFM_COFFEE,  coffee },
  { COFM_COFFEE,  tim2_finished, COFM_MILK,    milk   },
  { COFM_MILK,    tim2_finished, COFM_WAITING, finish },
  {-1, NULL, -1, NULL },
};

/*-----------------Temperature FSM--------------------------------------*/
static int pressed(fsm_t* this){
	if(flag_sensor){
		flag_sensor=0;
		return 1;
	}
	else{
		return 0;
	}
}
static void start_adc(fsm_t* this){
	HAL_ADC_Start(&hadc);
}
static int temp_high (fsm_t* this) {
	int is_high = 0;
	if (__HAL_ADC_GET_FLAG(&hadc, ADC_FLAG_EOC)){
		temperature = (((int32_t)HAL_ADC_GetValue(&hadc)*VDD_APPLI/VDD_CALIB)- (int32_t) *TEMP30_CAL_ADDR );
		temperature = temperature * (int32_t)(110 - 30);
		temperature = temperature / (int32_t)(*TEMP110_CAL_ADDR - *TEMP30_CAL_ADDR);
		temperature = temperature + 30;
		if(temperature>=0){
			is_high = 1;
		}
	}
	return is_high;
 }


static void led_on(fsm_t * this){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
}



static fsm_trans_t temperaturem[] = {
  {TEMP_IDLE, pressed, TEMP_SENSORING, start_adc },
  {TEMP_SENSORING, temp_high, TEMP_IDLE, led_on },
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

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of money */
  osMutexDef(money);
  moneyHandle = osMutexCreate(osMutex(money));

  /* Create the FSM�s */
  fsm_t* wallet_fsm = fsm_new (coinsm);
  fsm_t* coffee_fsm = fsm_new (cofm);
  fsm_t* temperature_fsm = fsm_new (temperaturem);
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of coffeeTask */
  osThreadDef(coffeeTask, StartCoffeeTask, osPriorityNormal, 0, 128);
  coffeeTaskHandle = osThreadCreate(osThread(coffeeTask), coffee_fsm);

  /* definition and creation of walletTask */
  osThreadDef(walletTask, StartWalletTask, osPriorityAboveNormal, 0, 128);
  walletTaskHandle = osThreadCreate(osThread(walletTask), wallet_fsm);

  /* definition and creation of temperatureTask */
  osThreadDef(temperatureTask, StartSensorTask, osPriorityBelowNormal, 0, 128);
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
  htim2.Init.Period = 4999;
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
  htim3.Init.Prescaler = 48000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, START_LED_Pin|COFFEE_LED_Pin|MILK_LED_Pin|FINISH_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SENSOR_BUTTON_Pin COIN_BUTTON_Pin START_BUTTON_Pin CANCELL_BUTTON_Pin */
  GPIO_InitStruct.Pin = SENSOR_BUTTON_Pin|COIN_BUTTON_Pin|START_BUTTON_Pin|CANCELL_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : START_LED_Pin COFFEE_LED_Pin MILK_LED_Pin FINISH_LED_Pin */
  GPIO_InitStruct.Pin = START_LED_Pin|COFFEE_LED_Pin|MILK_LED_Pin|FINISH_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

	if(GPIO_Pin == COIN_BUTTON_Pin){
		flag_coin_button = 1;
	}
	if(GPIO_Pin == START_BUTTON_Pin){
		flag_start_button = 1;
		}
	if(GPIO_Pin == CANCELL_BUTTON_Pin){
		flag_cancell_button = 1;
		}
	if(GPIO_Pin == SENSOR_BUTTON_Pin){
		flag_sensor = 1;
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

/* StartCoffeeTask function */
void StartCoffeeTask(void const * argument)
{
	fsm_t* pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = ( fsm_t * ) argument;
	xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
		fsm_fire(pcTaskName);
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
		}
}

/* StartWalletTask function */
void StartWalletTask(void const * argument)
{
	fsm_t* pcTaskName;
	TickType_t xLastWakeTime;
	pcTaskName = ( fsm_t * ) argument;
	xLastWakeTime = xTaskGetTickCount();

	for( ;; ){
		fsm_fire(pcTaskName);
		vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS( 100 ) );
		 }

}

/* StartSensorTask function */
void StartSensorTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorTask */
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	fsm_fire((fsm_t*)argument);
	vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(500) );
  }
  /* USER CODE END StartSensorTask */
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
	if (htim->Instance == TIM1) {
	    HAL_IncTick();
	  }
	if (htim->Instance == TIM2) {
		flag_tim2 = 1;
	    }
	if (htim->Instance == TIM3) {
		flag_tim3 = 1;
	    }
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
