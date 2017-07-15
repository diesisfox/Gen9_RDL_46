/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
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
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "can.h"
#include "serial.h"
#include "serial1.h"
#include "nodeMiscHelpers.h"
#include "nodeConf.h"
#include "../../CAN_ID.h"
#include "sd_io.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId ApplicationHandle;
osThreadId Can_ProcessorHandle;
osThreadId rxHousekeepHandle;
osThreadId SDLogHandle;
osThreadId radioTxHandle;
osThreadId cmdHandle;
osMessageQId mainCanTxQHandle;
osMessageQId mainCanRxQHandle;
osMessageQId SDLogCanQueueHandle;
osMessageQId radioTxQHandle;
osTimerId WWDGTmrHandle;
osTimerId HBTmrHandle;
osMutexId swMtxHandle;
osMutexId sdMtxHandle;
osMutexId rtcMtxHandle;
osMutexId vcpMtxHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t selfStatusWord;
extern Disk_drvTypeDef disk;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_SPI2_Init(void);
static void MX_RTC_Init(void);
void doApplication(void const * argument);
void doProcessCan(void const * argument);
void doRxHousekeep(void const * argument);
void doSDLog(void const * argument);
void doRadioTx(void const * argument);
void doCmd(void const * argument);
void TmrKickDog(void const * argument);
void TmrSendHB(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t toHex(uint8_t i){
	return (i<=9 ? '0'+i : 'A'+i-10);
}
void intToHex(uint32_t input, uint8_t *str, int length){
	for(int i=0; i<length; i++){
		str[length-1-i]=toHex(input&0x0F);
		input = input>>4;
	}
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	if (hcan == &hcan1)
		CAN1_TxCpltCallback(hcan);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	if (hcan == &hcan1)
		CAN1_RxCpltCallback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	if (hcan == &hcan1)
		CAN1_ErrorCallback(hcan);
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	selfStatusWord = INIT;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  hrtc.Instance = RTC;
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
    MX_RTC_Init();
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */
	Serial2_begin();

	/* init code for FATFS */


	Serial1_begin();

    bxCan_begin(&hcan1, &mainCanRxQHandle, &mainCanTxQHandle);
	bxCan_addMaskedFilterStd(0,0,0); // Filter: Status word group (ignore nodeID)
	bxCan_addMaskedFilterExt(0,0,0);
	// TODO: Set node-specific CAN filters

	Serial2_writeBuf("\n\nBooting... \n\n");

	SPI_IO_Attach(&hspi2);
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of swMtx */
  osMutexDef(swMtx);
  swMtxHandle = osMutexCreate(osMutex(swMtx));

  /* definition and creation of sdMtx */
  osMutexDef(sdMtx);
  sdMtxHandle = osMutexCreate(osMutex(sdMtx));

  /* definition and creation of rtcMtx */
  osMutexDef(rtcMtx);
  rtcMtxHandle = osMutexCreate(osMutex(rtcMtx));

  /* definition and creation of vcpMtx */
  osMutexDef(vcpMtx);
  vcpMtxHandle = osMutexCreate(osMutex(vcpMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of WWDGTmr */
  osTimerDef(WWDGTmr, TmrKickDog);
  WWDGTmrHandle = osTimerCreate(osTimer(WWDGTmr), osTimerPeriodic, NULL);

  /* definition and creation of HBTmr */
  osTimerDef(HBTmr, TmrSendHB);
  HBTmrHandle = osTimerCreate(osTimer(HBTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  osTimerStart(WWDGTmrHandle, WD_Interval);
  osTimerStart(HBTmrHandle, HB_Interval);
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of Application */
  osThreadDef(Application, doApplication, osPriorityAboveNormal, 0, 512);
  ApplicationHandle = osThreadCreate(osThread(Application), NULL);

  /* definition and creation of Can_Processor */
  osThreadDef(Can_Processor, doProcessCan, osPriorityNormal, 0, 512);
  Can_ProcessorHandle = osThreadCreate(osThread(Can_Processor), NULL);

  /* definition and creation of rxHousekeep */
  osThreadDef(rxHousekeep, doRxHousekeep, osPriorityHigh, 0, 256);
  rxHousekeepHandle = osThreadCreate(osThread(rxHousekeep), NULL);

  /* definition and creation of SDLog */
  osThreadDef(SDLog, doSDLog, osPriorityBelowNormal, 0, 2048);
  SDLogHandle = osThreadCreate(osThread(SDLog), NULL);

  /* definition and creation of radioTx */
  osThreadDef(radioTx, doRadioTx, osPriorityAboveNormal, 0, 512);
  radioTxHandle = osThreadCreate(osThread(radioTx), NULL);

  /* definition and creation of cmd */
  osThreadDef(cmd, doCmd, osPriorityNormal, 0, 512);
  cmdHandle = osThreadCreate(osThread(cmd), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxQ */
  osMessageQDef(mainCanTxQ, 64, Can_frame_t);
  mainCanTxQHandle = osMessageCreate(osMessageQ(mainCanTxQ), NULL);

  /* definition and creation of mainCanRxQ */
  osMessageQDef(mainCanRxQ, 64, Can_frame_t);
  mainCanRxQHandle = osMessageCreate(osMessageQ(mainCanRxQ), NULL);

  /* definition and creation of SDLogCanQueue */
  osMessageQDef(SDLogCanQueue, 64, Can_frame_t);
  SDLogCanQueueHandle = osMessageCreate(osMessageQ(SDLogCanQueue), NULL);

  /* definition and creation of radioTxQ */
  osMessageQDef(radioTxQ, 64, Can_frame_t);
  radioTxQHandle = osMessageCreate(osMessageQ(radioTxQ), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SPDIFRX;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_13TQ;
  hcan1.Init.BS2 = CAN_BS2_2TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN2 init function */
static void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_1TQ;
  hcan2.Init.BS2 = CAN_BS2_1TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = DISABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = DISABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* RTC init function */
static void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2){
  sTime.Hours = 0x13;
  sTime.Minutes = 0x27;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sDate.WeekDay = RTC_WEEKDAY_SUNDAY;
  sDate.Month = RTC_MONTH_JUNE;
  sDate.Date = 0x4;
  sDate.Year = 0x17;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR0,0x32F2);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_ENABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC5 PC6 
                           PC7 PC8 PC9 PC10 
                           PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA6 PA7 PA8 
                           PA9 PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* doApplication function */
void doApplication(void const * argument)
{
  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
	uint8_t startingBytes[2][9] = {
			{0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28},
			{0x29, 0x2a, 0x2c, 0x2d, 0x2e, 0x3a, 0x3b, 0x3c, 0x3e}
	};
	/* Infinite loop */
	for(;;){
		if(Serial1_available()){
//			if(getSelfState() == ACTIVE){
				static Can_frame_t newFrame;
				newFrame.isRemote = 0; //just to make sure
				uint8_t start = Serial1_read();
				for(uint8_t i=0; i<2; i++){
					for(uint8_t j=0; j<9; j++){
						if(start == startingBytes[i][j]){
							newFrame.isExt = i;
							newFrame.dlc = j;
							if(parseFrame(i, j, &newFrame)){
								bxCan_sendFrame(&newFrame);
								break;
							}
						}
					}
				}
				Serial2_write(Serial1_available());
//			}else{
//				Serial1_read();
//			}
		}else{
			osDelay(1);
		}
	}
  /* USER CODE END 5 */ 
}

/* doProcessCan function */
void doProcessCan(void const * argument)
{
  /* USER CODE BEGIN doProcessCan */
	/* Infinite loop */
	for(;;){
		static Can_frame_t newFrame;
		xQueueReceive(mainCanRxQHandle, &newFrame, portMAX_DELAY);
        xQueueSend(SDLogCanQueueHandle, &newFrame, 0);
        xQueueSend(radioTxQHandle, &newFrame, 0);

		uint32_t canID = newFrame.id;	// canID container //TODO why byte
		if(canID == p2pOffset || canID == selfNodeID + p2pOffset){
			// Multicast or unicast command received!
			taskENTER_CRITICAL();
			executeCommand(newFrame.Data[0]);
			taskEXIT_CRITICAL();
			// Note: Any application-level messages should be either mutex protected or passed via queue!
		}
	}
  /* USER CODE END doProcessCan */
}

/* doRxHousekeep function */
void doRxHousekeep(void const * argument)
{
  /* USER CODE BEGIN doRxHousekeep */
	static int bamboozle;
	bamboozle = 0;
	/* Infinite loop */
	for(;;){
		if(hcan1.State != HAL_CAN_STATE_BUSY_RX0 && hcan1.State != HAL_CAN_STATE_BUSY_TX_RX0){
			bamboozle++;
		}else{
			bamboozle = 0;
		}
		if(bamboozle > 8){
			HAL_CAN_Receive_IT(&hcan1, 0);
		}

		if(bamboozle > 12){
//			NVIC_SystemReset();
            HAL_NVIC_SystemReset();
		}

		osDelay(17);    //nice prime number
	}
  /* USER CODE END doRxHousekeep */
}

/* doSDLog function */
void doSDLog(void const * argument)
{
  /* USER CODE BEGIN doSDLog */
#define f_writeBuf(a,b,c) f_write((a),(b),(sizeof(b)-1),(c))
	static Can_frame_t newFrame;
	static FIL newFIL;
	static FATFS newFS;
	static uint8_t SD_Path[16];
	static UINT bw;
	static FRESULT ret;
	static RTC_DateTypeDef newDate;
	static RTC_TimeTypeDef newTime;

	static uint8_t dir0Name[] = "RDL_LOGS";
	static uint8_t dir1Name[] = "20xx";
	static uint8_t dir2Name[] = "xx";
	static uint8_t dir3Name[] = "xx";
	static uint8_t fileName[] = "xx_xx_xx.log";
	static uint8_t truemsg[] = "true";
	static uint8_t falsemsg[] = "false";
	static uint8_t stdidmsg[] = "xxx";
	static uint8_t extidmsg[] = "xxxxxxxx";
	static uint8_t datamsg[] = ",\"xx\"";
	static uint8_t charBuf;
	static uint8_t timeStampBuf[] = "xx:xx:xx";
    
    static Can_frame_t dcFrame;
    dcFrame.dlc = 0;
    dcFrame.id = 0x700;
    dcFrame.isExt = 0;
    dcFrame.isRemote = 0;

  /* init code for FATFS */
  MX_FATFS_Init();

	for(;;){
        xSemaphoreTake(rtcMtxHandle, portMAX_DELAY);
        HAL_RTC_GetTime(&hrtc, &newTime, RTC_FORMAT_BCD);
        HAL_RTC_GetDate(&hrtc, &newDate, RTC_FORMAT_BCD);
        xSemaphoreGive(rtcMtxHandle);
        fileName[0] = '0' + (newTime.Hours >> 4);
        fileName[1] = '0' + (newTime.Hours & 0xf);
        fileName[3] = '0' + (newTime.Minutes >> 4);
        fileName[4] = '0' + (newTime.Minutes & 0xf);
        fileName[6] = '0' + (newTime.Seconds >> 4);
        fileName[7] = '0' + (newTime.Seconds & 0xf);
        dir1Name[2] = '0' + (newDate.Year >> 4);
        dir1Name[3] = '0' + (newDate.Year & 0xf);
        dir2Name[0] = '0' + (newDate.Month >> 4);
        dir2Name[1] = '0' + (newDate.Month & 0xf);
        dir3Name[0] = '0' + (newDate.Date >> 4);
        dir3Name[1] = '0' + (newDate.Date & 0xf);

        ret = f_mount(0, SD_Path, 0);
        disk = (Disk_drvTypeDef){0};
        newFS = (FATFS){0};
        MX_FATFS_Init();
		ret = f_mount(&newFS, SD_Path, 0);
		ret = f_mkdir(dir0Name);
		if(ret!=FR_OK && ret!=FR_EXIST){
			osDelay(100);
			continue;
//            bxCan_sendFrame(&dcFrame);
		}
		ret = f_chdir(dir0Name);
		ret = f_mkdir(dir1Name);
		ret = f_chdir(dir1Name);
		ret = f_mkdir(dir2Name);
		ret = f_chdir(dir2Name);
		ret = f_mkdir(dir3Name);
		ret = f_chdir(dir3Name);
		ret = f_open(&newFIL, fileName, FA_CREATE_ALWAYS | FA_WRITE);

		uint8_t first = 1;
		f_writeBuf(&newFIL, "{\"entries\":[\n\n]}", &bw);
		f_sync(&newFIL);

		/* Infinite loop */
		for(;;){
			xQueueReceive(SDLogCanQueueHandle, &newFrame, portMAX_DELAY);

			xSemaphoreTake(rtcMtxHandle, portMAX_DELAY);
			HAL_RTC_GetTime(&hrtc, &newTime, RTC_FORMAT_BCD);
			HAL_RTC_GetDate(&hrtc, &newDate, RTC_FORMAT_BCD);
			xSemaphoreGive(rtcMtxHandle);

			timeStampBuf[0] = '0' + (newTime.Hours >> 4);
			timeStampBuf[1] = '0' + (newTime.Hours & 0xf);
			timeStampBuf[3] = '0' + (newTime.Minutes >> 4);
			timeStampBuf[4] = '0' + (newTime.Minutes & 0xf);
			timeStampBuf[6] = '0' + (newTime.Seconds >> 4);
			timeStampBuf[7] = '0' + (newTime.Seconds & 0xf);

			ret = f_lseek(&newFIL, f_tell(&newFIL) - 3);
			if(!first) f_writeBuf(&newFIL, ",\n", &bw);
			ret |= f_writeBuf(&newFIL, "{\"timestamp\":\"", &bw);
			ret |= f_writeBuf(&newFIL, timeStampBuf, &bw);
			ret |= f_writeBuf(&newFIL, "\",\"type\":\"frame\",\"ide\":", &bw);
			newFrame.isExt ? f_writeBuf(&newFIL, truemsg, &bw) : f_writeBuf(&newFIL, falsemsg, &bw);
			ret |= f_writeBuf(&newFIL, ",\"rtr\":", &bw);
			newFrame.isRemote ? f_writeBuf(&newFIL, truemsg, &bw) : f_writeBuf(&newFIL, falsemsg, &bw);
			ret |= f_writeBuf(&newFIL, ",\"dlc\":", &bw);
			charBuf = toHex(newFrame.dlc);
			ret |= f_write(&newFIL, &charBuf, 1, &bw);
			ret |= f_writeBuf(&newFIL, ",\"id\":\"", &bw);
			if(newFrame.isExt){
				intToHex(newFrame.id, extidmsg, 8);
				ret |= f_writeBuf(&newFIL, extidmsg, &bw);
			}else{
				intToHex(newFrame.id, stdidmsg, 3);
				ret |= f_writeBuf(&newFIL, stdidmsg, &bw);
			}
			if(newFrame.isRemote){
				ret |= f_writeBuf(&newFIL, "\"}\n]}", &bw);
			}else{
				ret |= f_writeBuf(&newFIL, "\",\"data\":[", &bw);
				for(int i=0; i<newFrame.dlc; i++){
					intToHex(newFrame.Data[i], datamsg+2, 2);
					if(i==0){
						ret |= f_write(&newFIL, datamsg+1, sizeof(datamsg)-2, &bw);
					}else{
						ret |= f_writeBuf(&newFIL, datamsg, &bw);
					}
				}
				ret |= f_writeBuf(&newFIL, "]}\n]}", &bw);
			}
			ret |= f_sync(&newFIL);
			first=0;
			if(ret!=FR_OK){
                bxCan_sendFrame(&dcFrame);
				osDelay(100);
				break;
			}
		}
	}
  /* USER CODE END doSDLog */
}

/* doRadioTx function */
void doRadioTx(void const * argument)
{
  /* USER CODE BEGIN doRadioTx */
  static Can_frame_t newFrame;
  /* Infinite loop */
  for(;;)
  {
    xQueueReceive(radioTxQHandle, &newFrame, portMAX_DELAY);
    frameToBase64(&newFrame);
  }
  /* USER CODE END doRadioTx */
}

/* doCmd function */
void doCmd(void const * argument)
{
  /* USER CODE BEGIN doCmd */
	RTC_DateTypeDef newDate;
	RTC_TimeTypeDef newTime;
	uint8_t scanBuf[12];
	uint8_t timeBuf[] = "20?? ?? ?? ??:??:??\n";
	/* Infinite loop */
	for(;;){
		if(Serial2_available()){
			switch (Serial2_read()) {
			case 'T':
				xSemaphoreTake(vcpMtxHandle, portMAX_DELAY);
				Serial2_writeBuf("set rtc time:\ntype [yymmddhhmmss] pls\n");
				xSemaphoreGive(vcpMtxHandle);
				while (Serial2_available() < 12) {
					osDelay(100);
				}
				for (uint8_t i = 0; i < 12; i++) {
					scanBuf[i] = Serial2_read();
				}
				uint8_t valid = 1;
				for (uint8_t i = 0; i < 12; i++) {
					if(scanBuf[i] > '9' || scanBuf[i] < '0'){
						xSemaphoreTake(vcpMtxHandle, portMAX_DELAY);
						Serial2_writeBuf("you dun goofd\n");
						xSemaphoreGive(vcpMtxHandle);
						valid = 0;
						break;
					}
					scanBuf[i] -= '0';
				}
				if(valid){
					newDate.Year = scanBuf[0]<<4 | scanBuf[1];
					newDate.Month = scanBuf[2]<<4 | scanBuf[3];
					newDate.Date = scanBuf[4]<<4 | scanBuf[5];
					newTime.Hours = scanBuf[6]<<4 | scanBuf[7];
					newTime.Minutes = scanBuf[8]<<4 | scanBuf[9];
					newTime.Seconds = scanBuf[10]<<4 | scanBuf[11];
					xSemaphoreTake(rtcMtxHandle, portMAX_DELAY);
					HAL_RTC_SetTime(&hrtc, &newTime, RTC_FORMAT_BCD);
					HAL_RTC_SetDate(&hrtc, &newDate, RTC_FORMAT_BCD);
					xSemaphoreGive(rtcMtxHandle);
				}
                xSemaphoreTake(vcpMtxHandle, portMAX_DELAY);
                Serial2_writeBuf("time set!\n");
                xSemaphoreGive(vcpMtxHandle);
				break;
			case 't':
				xSemaphoreTake(rtcMtxHandle, portMAX_DELAY);
				HAL_RTC_GetTime(&hrtc, &newTime, RTC_FORMAT_BCD);
				HAL_RTC_GetDate(&hrtc, &newDate, RTC_FORMAT_BCD);
				xSemaphoreGive(rtcMtxHandle);
				timeBuf[2] = '0'+(newDate.Year>>4);
				timeBuf[3] = '0'+(newDate.Year&0xf);
				timeBuf[5] = '0'+(newDate.Month>>4);
				timeBuf[6] = '0'+(newDate.Month&0xf);
				timeBuf[8] = '0'+(newDate.Date>>4);
				timeBuf[9] = '0'+(newDate.Date&0xf);
				timeBuf[11] = '0'+(newTime.Hours>>4);
				timeBuf[12] = '0'+(newTime.Hours&0xf);
				timeBuf[14] = '0'+(newTime.Minutes>>4);
				timeBuf[15] = '0'+(newTime.Minutes&0xf);
				timeBuf[17] = '0'+(newTime.Seconds>>4);
				timeBuf[18] = '0'+(newTime.Seconds&0xf);
				xSemaphoreTake(vcpMtxHandle, portMAX_DELAY);
				Serial2_writeBuf("the rtc time is now: ");
				Serial2_writeBuf(timeBuf);
				xSemaphoreGive(vcpMtxHandle);
                break;
			default:
				xSemaphoreTake(vcpMtxHandle, portMAX_DELAY);
				Serial2_writeBuf("invalid command.\n");
				xSemaphoreGive(vcpMtxHandle);
				break;
			}
		}else{
			osDelay(100);
		}
	}
  /* USER CODE END doCmd */
}

/* TmrKickDog function */
void TmrKickDog(void const * argument)
{
  /* USER CODE BEGIN TmrKickDog */
	// CHECKED
	taskENTER_CRITICAL();
	HAL_WWDG_Refresh(&hwwdg);
	taskEXIT_CRITICAL();
  /* USER CODE END TmrKickDog */
}

/* TmrSendHB function */
void TmrSendHB(void const * argument)
{
  /* USER CODE BEGIN TmrSendHB */
	// CHECKED
	static Can_frame_t newFrame;

	//	newFrame.isExt = 0;
	//	newFrame.isRemote = 0;
	// ^ is initialized as 0

	if(getSelfState() == ACTIVE){
		// Assemble new heartbeat frame
		newFrame.id = selfNodeID + swOffset;
		newFrame.dlc = CAN_HB_DLC;
		for(int i=0; i<4; i++){
			newFrame.Data[3-i] = (selfStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		}
		bxCan_sendFrame(&newFrame);
#ifdef DEBUG
		static uint8_t hbmsg[] = "Heartbeat issued\n";
		Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
#endif
	}
	else if (getSelfState() == INIT){
		// Assemble new addition request (firmware version) frame
		newFrame.id = selfNodeID + fwOffset;
		newFrame.dlc = CAN_FW_DLC;
		for(int i=0; i<4; i++){
			newFrame.Data[3-i] = (firmwareString >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		}
		bxCan_sendFrame(&newFrame);
#ifdef DEBUG
		static uint8_t hbmsg[] = "Init handshake issued\n";
		Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
#endif
	}
	// No heartbeats sent in other states
  /* USER CODE END TmrSendHB */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
    HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
    HAL_Delay(200);
    HAL_NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */ 
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
