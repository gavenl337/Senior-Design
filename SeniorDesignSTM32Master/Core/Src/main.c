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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <string.h>
#include <stdio.h>
#include <stdbool.h> // I ADDED THINGS HERE !!!!!!!!
#include <stdlib.h>
#include <time.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void displayResults(int ADC_number, int i, int val);
void digitalPotWrite(int value);
int targetCheck(int val, int target, int i);
void DMATransferComplete(DMA_HandleTypeDef *hdma);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t adc_buf[ADC_BUF_LEN], adc[6];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	for (int i = 0; i<6; i++)
		{
			adc[i] = buffer[i];
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
	char uart_buf[127] = {'\0'};	//buffer for output data
	int uart_buf_len = {'\0'};


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	int target = 1024; //target analog bit value from the ADC
	uint8_t spiData[6];
	for(int j = 0; j < 6; j++){spiData[j] = 0x00;}
	int SPI_Transmit_Data_1;
	int SPI_Transmit_Data_2;
	int SPI_Transmit_Data_3;
	char deviceID[20] = "Unit One";
	char readingType[20] = "Ambient";
	int readingNumber = 0;
	int deviceID_Number = 1;

  //define sensor warming time
  #define SENS_WARMING_TIME 300 //approx. 5 minutes

  unsigned long lastMillis = 0; // I ADDED THINGS HERE !!!!!!!!
  int long sensor1ValuesA[21]; // I ADDED THINGS HERE !!!!!!!!
  int long sensor2ValuesA[21]; // I ADDED THINGS HERE !!!!!!!!
  int long sensor3ValuesA[21]; // I ADDED THINGS HERE !!!!!!!!
  int long sensor1ValuesB[21]; // I ADDED THINGS HERE !!!!!!!!
  int long sensor2ValuesB[21]; // I ADDED THINGS HERE !!!!!!!!
  int long sensor3ValuesB[21]; // I ADDED THINGS HERE !!!!!!!!

  char *valuePayload; // I ADDED THINGS HERE !!!!!!!!
  char str[80]; // how long does MQTT need to be?
  time_t t;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  //LowPower Sleep mode button-interrupt connection
  // LowPower.attachInterruptWakeup(BUTTON_PIN, wakeUp, CHANGE); // I ADDED THINGS HERE !!!!!!!! NEEDS TO BE CHANGED

   //GSM Setup
  // Serial.begin(115200); // I ADDED THINGS HERE !!!!!!!! NEEDS TO BE CHANGED
   //while (!Serial);
 //}

   // Optional, set the client id used for MQTT,
   // each device that is connected to the broker
   // must have a unique client id. The MQTTClient will generate
   // a client id for you based on the millis() value if not set
   //
  // mqttClient.setId("ISLGSM001"); // I ADDED THINGS HERE !!!!!!!! NEEDS TO BE CHANGED

   // Set the message callback, this function is
   // called when the MQTTClient receives a message
  // mqttClient.onMessage(onMessageReceived); // I ADDED THINGS HERE !!!!!!!! NEEDS TO BE CHANGED
 //}

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);	//set CS1 pin HIGH.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);	//set CS2 pin HIGH.
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//set CS3 pin HIGH.
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1){
	  	//Fast blinking - Blinking red light 4 times per second for 3 seconds indicating begining of sensor warmup
		for (int i = 0; i < 12; i++) { // I ADDED THINGS HERE !!!!!!!!
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!!
		  HAL_Delay(125); // I ADDED THINGS HERE !!!!!!!!
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // I ADDED THINGS HERE !!!!!!!!
		  HAL_Delay(125); // I ADDED THINGS HERE !!!!!!!!
		}

		//code to power up sensors here
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!!

		//Fast blinking - Blinking red light 1 times per second for 5 minute/s indicating sensor warming up
		 for (int i = 0; i < SENS_WARMING_TIME; i++) {
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!!
			 HAL_Delay(125); // I ADDED THINGS HERE !!!!!!!!
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // I ADDED THINGS HERE !!!!!!!!
			 HAL_Delay(875); // I ADDED THINGS HERE !!!!!!!!
		 }

		 //Fast Blinking (four times a second) Yellow for 5 sec warnning begining of data collection via MQTT
		   for (int i = 0; i < 20; i++) {
			   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!!
			   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!!
			   HAL_Delay(125); // I ADDED THINGS HERE !!!!!!!!
			   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // I ADDED THINGS HERE !!!!!!!!
			   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // I ADDED THINGS HERE !!!!!!!!
			   HAL_Delay(125); // I ADDED THINGS HERE !!!!!!!!
		   }

	  for(int measurement = 0; measurement < 10; measurement++){

		  //------------------------------Read SPI Data------------------------------//

		  spiData[0] = targetCheck(adc[0], target, spiData[0]);
		  spiData[1] = targetCheck(adc[1], target, spiData[1]);
		  spiData[2] = targetCheck(adc[2], target, spiData[2]);
		  SPI_Transmit_Data_1 = 0x00 | spiData[0];
		  SPI_Transmit_Data_2 = 0x00 | spiData[1];
		  SPI_Transmit_Data_3 = 0x00 | spiData[2];

		  //------------------------------Transmit SPI Data to 3 Digital Potentiometers------------------------------//

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);				//set CS1 pin LOW.
		  HAL_SPI_Transmit(&hspi1, (uint8_t *)&SPI_Transmit_Data_1, 1, 10); //handle SPI, Cast data to a 16 bit unsigned integer, 1 bytes of data, 10 ms delay
		  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);				//set CS1 pin HIGH.
		  HAL_Delay(100);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);				//set CS2 pin LOW.
		  HAL_SPI_Transmit(&hspi1, (uint8_t *)&SPI_Transmit_Data_2, 1, 10); //handle SPI, Cast data to a 16 bit unsigned integer, 1 bytes of data, 10 ms delay
		  while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);				//set CS2 pin HIGH.

		  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);				//set CS3 pin LOW.
		  //HAL_SPI_Transmit(&hspi1, (uint8_t *)&SPI_Transmit_Data_3, 1, 10); //handle SPI, Cast data to a 16 bit unsigned integer, 1 bytes of data, 10 ms delay
		  //while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		  //HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);				//set CS3 pin HIGH.

		  //------------------------------Send USART Data to Particle Boron Board------------------------------//

		  uart_buf_len =sprintf(uart_buf, "Unit %d, %d, %d, %s, %d\n", deviceID_Number, adc[0], adc[1], readingType, readingNumber);	  		//load print buffer with message
		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);	//print to terminal
		  readingNumber++;

		  //------------------------------Display Results to Terminal------------------------------//

		  uart_buf_len =sprintf(uart_buf, "Test #%d\n", measurement);	  		//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);	//print to terminal
		  for(int i = 0; i < 2; i++){											//iterate through SPI array and print results to terminal
			  displayResults(i+1, spiData[i], adc[i]);
		  }
		  HAL_Delay(1000);

	  }

	  char readingType[20] = "Breath";
	  for(int measurement = 0; measurement < 10; measurement++){
		  uart_buf_len =sprintf(uart_buf, "Unit %d, %d, %d, %s, %d\n", deviceID_Number, adc[0], adc[1], readingType, readingNumber);	  		//load print buffer with message
		  HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, uart_buf_len, 100);	//print to terminal
		  uart_buf_len =sprintf(uart_buf, "Unit %d, %d, %d, %s, %d\r \n", deviceID_Number, adc[0], adc[1], readingType, readingNumber);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal
		  readingNumber++;
	  	  HAL_Delay(1000);
	  }
	  readingNumber = 0;
	  deviceID_Number++;

	  //shows a red,yellow,green "get ready" sequence
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!! Red LED on
	  HAL_Delay(3000);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET); // I ADDED THINGS HERE !!!!!!!! Green LED on
	  HAL_Delay(3000);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET); // I ADDED THINGS HERE !!!!!!!! Red LED off
      //FOR USER: BREATHE INTO THE SENSOR

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET); // mosfet pin low (stops current flow to heater pins)
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET); // Green LED off

	  HAL_Delay(1500);
  }
}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_SYSCLK;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 16;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Heater_GPIO_Port, Heater_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, CS3_Pin|CS2_Pin|CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Heater_Pin */
  GPIO_InitStruct.Pin = Heater_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Heater_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS3_Pin CS2_Pin CS1_Pin */
  GPIO_InitStruct.Pin = CS3_Pin|CS2_Pin|CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Dip5_Pin Dip4_Pin Dip3_Pin Dip2_Pin
                           Dip1_Pin */
  GPIO_InitStruct.Pin = Dip5_Pin|Dip4_Pin|Dip3_Pin|Dip2_Pin
                          |Dip1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void displayResults(int ADC_number, int i, int val){
  char uart_buf[350] = {'\0'};	//buffer for output data
  int uart_buf_len = {'\0'};
  float voltage = (3.3/4096) * val;
  float potValue = (10000/128) * i;

  uart_buf_len =sprintf(uart_buf, "\tADC: #%d\n \
		  	  	  	  	  	  	  \r\t\tPotentiometer Value (0 - 128): %d\n \
		  	  	  	  	  	  	  \r\t\tResistance: .................. %.0f Ohms\n \
		  	  	  	  	  	  	  \r\t\tADC Value (0 - 4096): ........ %d\n \
		  	  	  	  	  	  	  \r\t\tADC Voltage: ................. %.2fV\n\n\r", ADC_number, i, potValue, val, voltage);	  	//load print buffer with message
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal
}

int targetCheck(int val, int target, int i){
	//char uart_buf[50] = {'\0'};	//buffer for output data
	//int uart_buf_len = {'\0'};

	//uart_buf_len =sprintf(uart_buf, "\rpotentiometer bit value inside function = %d\r\n", i);	  	//load print buffer with message
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

	//check for distance val is from the target
	//if the analog value is greater than 50% of the target value, decrease 'i' by 50
		/*if (val > 1.50 * target)
			i = i - 50;
		//if the analog value is greater than 40% of the target value, decrease 'i' by 40
		else if (val > 1.40 * target)
			i = i - 40;
		//if the analog value is greater than 30% of the target value, decrease 'i' by 30
		else if (val > 1.30 * target)
			i = i - 30;
			*/
		//if the analog value is greater than 20% of the target value, decrease 'i' by 20
		if (val > 1.20 * target)
			i = i - 20;
		//if the analog value is greater than 10% of the target value, decrease 'i' by 10
		else if (val > 1.10 * target)
			i = i - 10;
		//if the analog value is greater than 20% of the target value, decrease 'i' by 5
		if (val > 1.04 * target)
			i = i - 5;
		//if the analog value is greater than 20% of the target value, decrease 'i' by 1
		else if (val > target)
			i = i - 1;
		//--------------------------------------------------------------------------------------------------------------
		//if the analog value is greater than 20% of the target value, decrease 'i' by 50
		else if (val < (1- 0.50) * target){
			i = i + 50;
		}
		else if (val < (1- 0.04) * target){
			i = i + 5;
		}
		//if the analog value is greater than 40% of the target value, decrease 'i' by 50
		else if (val < (1- 0.10) * target){
			i = i + 10;
		}
		//if the analog value is greater than 30% of the target value, decrease 'i' by 50
		else if (val < (1- 0.20) * target){
			i = i + 20;
		}
		//if the analog value is greater than 20% of the target value, decrease 'i' by 50
		else if (val < (1- 0.30) * target){
			i = i + 30;
		}
		//if the analog value is greater than 10% of the target value, decrease 'i' by 50
		else if (val < (1- 0.40) * target){
			i = i + 40;
		}
		else if (val < target){
			i = i + 1;
		}
		else
			i = i;

	//dataString = createCSV(val,val,val,tNum);
	if(i > 128){
	  //uart_buf_len =sprintf(uart_buf, "Entered if Statement about 128\n");	  	//load print buffer with message
	  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal
	  i = 128;
	}
	if(i < 0){
	  //uart_buf_len =sprintf(uart_buf, "Entered if Statement below 0\n");	  	//load print buffer with message
	  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal
	  i = 0;
	}
	//uart_buf_len =sprintf(uart_buf, "\rpot value being returned after algorithm = %d\r\n", i);	  	//load print buffer with message
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal
	return i;
}

void digitalPotWrite(int value){
	//char uart_buf[50] = {'\0'};	//buffer for output data
	//int uart_buf_len = {'\0'};

	//uart_buf_len =sprintf(uart_buf, "\rWriting %d to the ADC\r\n\n", value);	  	//load print buffer with message
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);				//print to terminal

	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	//set CS1 pin LOW.
	//HAL_SPI_Transmit(&hspi1, (uint8_t *)&value, 1, 400); //handle SPI, Cast data to a 16 bit unsigned integer, 2 bytes of data, 400 ms delay
	//HAL_Delay(100);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);	//set CS1 pin HIGH.
	//HAL_Delay(100);
	}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc){
	/*char uart_buf[50] = {'\0'};	//buffer for output data
	int uart_buf_len = {'\0'};
	uart_buf_len =sprintf(uart_buf, "\rFirt ADC Handle\r\n\n");	  	//load print buffer with message
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);*/
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	adc[0] = adc_buf[0];
	adc[1] = adc_buf[1];
	adc[2] = adc_buf[2];
	adc[3] = adc_buf[3];
	adc[4] = adc_buf[4];
	adc[5] = adc_buf[5];
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	char uart_buf[50] = {'\0'};	//buffer for output data
	int uart_buf_len = {'\0'};
	uart_buf_len =sprintf(uart_buf, "\rSecond ADC Handle\r\n\n");	  	//load print buffer with message
	HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);
}*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	  /* User can add his own implementation to report the HAL error return state */
	  __disable_irq();
	  while (1){
	  }
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
