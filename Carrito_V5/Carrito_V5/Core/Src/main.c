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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PAYLOAD				22


#define PWM_PIN GPIO_PIN_0
#define PWM_PIN_2 GPIO_PIN_2
#define PWM_PORT GPIOA

SPI_HandleTypeDef hspi2;


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim9;
TIM_OC_InitTypeDef sConfigOC;



/* USER CODE BEGIN PV */
nrf24 nrfRx;

uint8_t rxAddr1[] = { 0xEA, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t rxAddr2[] = { 0xEB, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t rxAddr3[] = { 0xEC, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t rxAddr4[] = { 0xED, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t rxAddr5[] = { 0xEE, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t rxAddr6[] = { 0xEF, 0xDD, 0xCC, 0xBB, 0xAA };
uint8_t rxData[PAYLOAD];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);


void Avanza(void);
void Giro_I(void);
void Retroceso(void);
void Giro_D(void);
void Alto(void);


void PWM_1(void);
void PWM_2(void);




/* USER CODE BEGIN PFP */
void readData(void);

int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();



	 MX_TIM2_Init();
	  PWM_1();
	  PWM_2();
	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();

	/* USER CODE BEGIN 2 */
	// MODULE SETTINGS ----------------------------------------------
	 nrfRx.CE_port = GPIOB;
	 nrfRx.CE_pin = GPIO_PIN_7;
	 nrfRx.CSN_port = GPIOB;
	 nrfRx.CSN_pin =GPIO_PIN_12;
	 nrfRx.IRQ_port = GPIOB;
	 nrfRx.IRQ_pin = GPIO_PIN_0;
	 nrfRx.hSPIx = &hspi2;

	nrf24_init(&nrfRx);
	nrf24_setDataRate(&nrfRx, _250kbs);
	nrf24_setPALevel(&nrfRx, high);

	nrf24_setRxPipe(&nrfRx, rxAddr1, 0, PAYLOAD);
	nrf24_setRxPipe(&nrfRx, rxAddr2, 1, PAYLOAD);
	nrf24_setRxPipe(&nrfRx, rxAddr4, 2, PAYLOAD);
	nrf24_setRxPipe(&nrfRx, rxAddr3, 3, PAYLOAD);
	nrf24_setMode(&nrfRx, rxMode);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		for (int i = 0; i < 4; i++) {
			if (isDataAvailable(&nrfRx, i) == 1) readData();
		}
		HAL_Delay(10);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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





static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

void PWM_1(void)
{
		HAL_Init();
	    SystemClock_Config();

	    GPIO_InitTypeDef GPIO_InitStruct = {0};
	    __HAL_RCC_GPIOA_CLK_ENABLE();

	    GPIO_InitStruct.Pin = PWM_PIN;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // Cambiado a alta frecuencia
	    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
	    HAL_GPIO_Init(PWM_PORT, &GPIO_InitStruct);

	    __HAL_RCC_TIM2_CLK_ENABLE();

	    htim2.Instance = TIM2;
	    htim2.Init.Prescaler = 0;  // Cambiado a 0 para la máxima frecuencia
	    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	    htim2.Init.Period = 65535;  // Cambiado al máximo período de 16 bits
	    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	    {
	        Error_Handler();
	    }

	    sConfigOC.OCMode = TIM_OCMODE_PWM1;
	    sConfigOC.Pulse = 65535;  // Cambiado al máximo pulso
	    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	    {
	        Error_Handler();
	    }

	    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}


void PWM_2(void)
{
	HAL_Init();
	SystemClock_Config();

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = PWM_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_TIM9;
	HAL_GPIO_Init(PWM_PORT, &GPIO_InitStruct);

	__HAL_RCC_TIM9_CLK_ENABLE();

	htim9.Instance = TIM9;
	htim9.Init.Prescaler = 0;
	htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim9.Init.Period = 65535;
	htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
	{
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 65535;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
}





/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi2.Init.CRCPolynomial = 7;
	//hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	//hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}


}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {

	 GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	  __HAL_RCC_GPIOB_CLK_ENABLE();

	  /*Configure GPIO pin Output Level */
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	  /*Configure GPIO pin : PB12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_12|GPIO_PIN_0|GPIO_PIN_8|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_9;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void readData(void) {
	nrf24_Receive(&nrfRx, rxData, PAYLOAD);

	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	if (rxData[0]=='I')
	{
		 Avanza();
	}
	if (rxData[1]=='D')
	{
		 Giro_I();
	}
	if (rxData[2]=='A')
	{
		 Giro_D();
	}
	if (rxData[3]=='R')
	{
		Retroceso();
	}
	if (rxData[4]=='P')
	{
		Alto();
	}

}


void Avanza_A(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void Avanza_B(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
}

void Avanza(void)
{
	Avanza_A();
	Avanza_B();
}


void Giro_I_A(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}



void Giro_I_B(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}


void Giro_I(void)
{
	Giro_I_A();
	Giro_I_B();
}


void Giro_D_A(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}

void Giro_D_B(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void Giro_D(void)
{
	Giro_D_A();
	Giro_D_B();
}

void Retroceso_A(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
}

void Retroceso_B(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
}

void Retroceso(void)
{
	Retroceso_A();
	Retroceso_B();
}

void Alto_A(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}
void Alto_B(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

}

void Alto(void)
{
	Alto_A();
	Alto_B();
}



//-------------------------------------------------------
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
#endif /* USE_FULL_ASSERT */
