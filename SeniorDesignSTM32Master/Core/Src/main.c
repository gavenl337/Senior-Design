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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char uart_buf[50] = {'\0'};	//buffer for output data
	int uart_buf_len = {'\0'};

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	//byte address = 0x00;

	//int i = 64; //starting value of 540
	int target = 1024; //target analog bit value from the ADC
	int tNum = 1;
	//int NOP = 0b00000000;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);	//set CS1 pin HIGH.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t spiData[6];

  for(int j = 0; j < 6; j++){
	  spiData[j] = 0x00;
  }

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
  while (1){
	  for(int measurement = 0; measurement <= 10; measurement++){

		  uart_buf_len =sprintf(uart_buf, "\'val 1\'= %ld\r\n", adc[0]);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  uart_buf_len =sprintf(uart_buf, "\'val 2\'= %ld\r\n", adc[1]);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  uart_buf_len =sprintf(uart_buf, "\'val 3\'= %ld\r\n", adc[2]);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  uart_buf_len =sprintf(uart_buf, "\'val 4\'= %ld\r\n", adc[3]);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  uart_buf_len =sprintf(uart_buf, "\'val 5\'= %ld\r\n", adc[4]);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  uart_buf_len =sprintf(uart_buf, "\'val 6\'= %ld\r\n\n", adc[5]);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  //uart_buf_len =sprintf(uart_buf, "\'spiData\' Before Conversion = %d\r\n", spiData[0]);	  	//load print buffer with message
		  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal


		  spiData[0] = targetCheck(adc[0], target, spiData[0]);
		  int SPI_Transmit_Data =  0x00 | spiData[0];
		  //iterate through SPI array and load with values
		  //for(int i = 0; i < 6; i++){
			  //spiData[i] = targetCheck(adc[i], target, spiData[i]);
		  //}

		  //uart_buf_len =sprintf(uart_buf, "\'spiData\' After Conversion = %d\r\n", spiData[0]);	  	//load print buffer with message
		  //HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

		  uart_buf_len =sprintf(uart_buf, "Test #%d\n", measurement);	  	//load print buffer with message
		  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal
		  //iterate through SPI array and print results to terminal
		  for(int i = 0; i < 6; i++){
			  displayResults(i+1, spiData[i], adc[i]);
		  }

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	//set CS1 pin LOW.
		  //HAL_Delay(1);
		  HAL_SPI_Transmit(&hspi1, (uint8_t *)&SPI_Transmit_Data, 1, 0); //handle SPI, Cast data to a 16 bit unsigned integer, 2 bytes of data, 400 ms delay
		  //HAL_Delay(1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);	//set CS1 pin HIGH.

		  //digitalPotWrite(i);
		  HAL_Delay(1000);
		//strcpy((char*)buf, "Hello!\r\n");
		//HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
		//HAL_Delay(500);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		  HAL_GPIO_TogglePin (GPIOA, GPIO_PIN_5);
		  HAL_Delay(500);
		  //Using DMA to scan ADC values. Array adc[] holds the values
		  //from the 6 ADC inputs. sensor1=adc[0] sensor2=adc[1] etc.
	  	  }
	tNum++;
	HAL_Delay(3000);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
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
  char uart_buf[50] = {'\0'};	//buffer for output data
  int uart_buf_len = {'\0'};
  float voltage = (3.3/4096) * val;
  float potValue = (10000/128) * i;

  //sprintf(uart_buf, "Test # %d", measurement);	  			//load print buffer with message
  //HAL_UART_Transmit(&huart2, (uint8_t *)MSG, uart_buf_len, 100);		//print to terminal

  uart_buf_len =sprintf(uart_buf, "\tADC: #%d\n", ADC_number);	  	//load print buffer with message
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

  uart_buf_len =sprintf(uart_buf, "\r\t\tPotentiometer Value (0 - 128): %d\n", i);	//load print buffer with message
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

  uart_buf_len =sprintf(uart_buf, "\r\t\tResistance: .................. %.0f Ohms\n", potValue);	//load print buffer with message
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

  uart_buf_len =sprintf(uart_buf, "\r\t\tADC Value (0 - 4096): ........ %d\n", val);	  		//load print buffer with message
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

  uart_buf_len =sprintf(uart_buf, "\r\t\tADC Voltage: ................. %.2fV\n\n\r", voltage);	  			//load print buffer with message
  HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

}

int targetCheck(int val, int target, int i){
	//char uart_buf[50] = {'\0'};	//buffer for output data
	//int uart_buf_len = {'\0'};

	//uart_buf_len =sprintf(uart_buf, "\rpotentiometer bit value inside function = %d\r\n", i);	  	//load print buffer with message
	//HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, uart_buf_len, 100);		//print to terminal

	//check for distance val is from the target
	//if the analog value is greater than 50% of the target value, decrease 'i' by 50
		if (val > 1.50 * target)
			i = i - 50;
		//if the analog value is greater than 40% of the target value, decrease 'i' by 40
		else if (val > 1.40 * target)
			i = i - 40;
		//if the analog value is greater than 30% of the target value, decrease 'i' by 30
		else if (val > 1.30 * target)
			i = i - 30;
		//if the analog value is greater than 20% of the target value, decrease 'i' by 20
		else if (val > 1.20 * target)
			i = i - 20;
		//if the analog value is greater than 10% of the target value, decrease 'i' by 10
		else if (val > 1.10 * target)
			i = i - 10;
		//if the analog value is greater than 20% of the target value, decrease 'i' by 5
		else if (val > 1.04 * target)
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
