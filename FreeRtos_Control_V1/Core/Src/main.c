/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
#include <stdio.h>
#include "cmsis_os.h"

osThreadId defaultTaskHandle;

osThreadId myTask02Handle;
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;


uint32_t adc_buf[2];
uint16_t valor_adc[2];

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
nrf24 nrfTx;

uint8_t txAddr[] = { 0xEA, 0xDD, 0xCC, 0xBB, 0xAA };
//uint8_t txData[22] = "N";


uint8_t txData[22] = "I7848"; ///Avanza
uint8_t txData1[22] ="4D789";///Giro a la Izquierda
uint8_t txData2[22] ="59A17";///Giro a la Derecha
uint8_t txData3[22] ="367R3";///Retroceso
uint8_t txData4[22] ="1235P";///Alto total
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin); ///boton interrupcion
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);





int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();

  HAL_ADC_Start_DMA(&hadc1, adc_buf, 2);
  HAL_ADC_Start_IT(&hadc1);
  /* USER CODE BEGIN 2 */
	// MODULE SETTINGS ----------------------------------------------
	 nrfTx.CE_port = GPIOB;
	 nrfTx.CE_pin = GPIO_PIN_7;
	 nrfTx.CSN_port = GPIOB;
	 nrfTx.CSN_pin =GPIO_PIN_12;
	 nrfTx.IRQ_port = GPIOB;
	 nrfTx.IRQ_pin = GPIO_PIN_0;
	 nrfTx.hSPIx = &hspi2;

	 nrf24_init(&nrfTx);
	 nrf24_setTxAddr(&nrfTx, txAddr);

	  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);



	  osThreadDef(myTask02, StartTask02, osPriorityHigh, 0, 128);
	  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);





		  /* Start scheduler */
		  osKernelStart();



	while (1) {


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig = {0};



  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}


static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
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
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

//void StartDefaultTask(void const * argument)
//{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  //for(;;)
  //{

  	  //HAL_ADC_Start_IT(&hadc1);
  	  //nrf24_setMode(&nrfTx, txMode);

	  //if((valor_adc[0]>=2000) && (valor_adc[0]<=3000))
	  //{
		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		  //if (nrf24_Transmit(&nrfTx, txData, sizeof(txData)) == 1) {
			  //nrf24_setMode(&nrfTx, standby);
			  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		  //}
	  //}
	  //osDelay(10);

  //}
  /* USER CODE END 5 */
//}

void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  	    HAL_ADC_Start_IT(&hadc1);
	        nrf24_setMode(&nrfTx, txMode);
	        if ((valor_adc[1]>=1700) && (valor_adc[1]<=3000))
	        	  	  {
	        	  		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	        	  		  if (nrf24_Transmit(&nrfTx, txData4, sizeof(txData4)) == 1) {
	        	  			  nrf24_setMode(&nrfTx, standby);
	        	  			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	        	  		  }
	        	  		  //HAL_Delay(1000);
	        	  	  }





	        if((valor_adc[1]>=3100))
	        	  	  {

	        	  		  if (nrf24_Transmit(&nrfTx, txData1, sizeof(txData1)) == 1) {
	        	    		  nrf24_setMode(&nrfTx, standby);

	        	  		  }
	        	  		  //HAL_Delay(1000);
	        	  	  }

	       if((valor_adc[1]<=1000))
	        	  	{

	        	  		if (nrf24_Transmit(&nrfTx, txData2, sizeof(txData2)) == 1) {
	        	  		    nrf24_setMode(&nrfTx, standby);

	        	  		 }
	        	  		  		  //HAL_Delay(1000);
	        	  	}

	        	  	osDelay(100);///////////////////////////////////////////////////////////////
  }
  /* USER CODE END 5 */
}

void StartTask02(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {

	  	  	  	//HAL_ADC_Start_IT(&hadc1);
		        //nrf24_setMode(&nrfTx, txMode);
		        //if((valor_adc[0]>=1000) && (valor_adc[0]<=3000))
		        	  	  //{
		        	  		  //if (nrf24_Transmit(&nrfTx, txData4, sizeof(txData4)) == 1) {
		        	  			  //nrf24_setMode(&nrfTx, standby);
		        	  			  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

		        	  		  //}
		        	  	  //}

		        if((valor_adc[0]>=3100))
		        	  	  {
		        	  		  if (nrf24_Transmit(&nrfTx, txData, sizeof(txData)) == 1)
		        	  		  {
		        	  			  nrf24_setMode(&nrfTx, standby);
		        	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

		        	  		  }
		        	  	  }


		        if((valor_adc[0]<=1000))
		        	  	  {

		        	  		  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
		        	  		  if (nrf24_Transmit(&nrfTx, txData3, sizeof(txData3)) == 1) {
		        	  			  nrf24_setMode(&nrfTx, standby);

		        	  		  }
		        	  		  //HAL_Delay(1000);
		        	  	  }

		        osDelay(100);///////////////////////////////////////////////////////////////
  }
  /* USER CODE END 5 */
}




void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		valor_adc[0] = (uint16_t)adc_buf[0];
		valor_adc[1] = (uint16_t)adc_buf[1];

	}
}





static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	  /*Configure GPIO pin : PB12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
	while (1) {
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
#endif /* USE_FU*/


