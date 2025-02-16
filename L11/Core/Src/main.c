/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
// KB
uint8_t col = 0;	// current column to be read
GPIO_PinState prev[4][4] = {{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET}, // transposed keyboard matrix of previous values
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET}
};
GPIO_PinState prev2[4][4] = {{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET}, // transposed keyboard matrix of the values read 2 cycles before
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET},
		{GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_SET}
};
GPIO_PinState col_activation[4][4] = {{GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET}, // values to send to column GPIO pins for activation
		{GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET},
		{GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET},
		{GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET}
};
char character[4][4] = {{'F', 'B', '7', '3'},	// character associated with each key
		{'E', 'A', '6', '2'},
		{'D', '9', '5', '1'},
		{'C', '8', '4', '0'}
};
//int BaudElapsedFlag = 0;

// LED
uint8_t matrix[5][2] = {
		{0, 16},
		{0, 8},
		{0, 4},
		{0, 2},
		{0, 1}
};

uint8_t matrix_QM[5][2] = {
		{32, 16},
		{64, 8},
		{69, 4},
		{72, 2},
		{48, 1}
};

uint8_t matrix_0[5][2] = {
		{62, 16},
		{113, 8},
		{73, 4},
		{71, 2},
		{62, 1}
};

uint8_t matrix_1[5][2] = {
		{16, 16},
		{33, 8},
		{127, 4},
		{1, 2},
		{0, 1}
};

uint8_t matrix_2[5][2] = {
		{33, 16},
		{67, 8},
		{69, 4},
		{73, 2},
		{49, 1}
};

uint8_t matrix_3[5][2] = {
		{34, 16},
		{65, 8},
		{73, 4},
		{73, 2},
		{54, 1}
};

uint8_t matrix_4[5][2] = {
		{120, 16},
		{8, 8},
		{8, 4},
		{8, 2},
		{127, 1}
};

uint8_t matrix_5[5][2] = {
		{114, 16},
		{81, 8},
		{81, 4},
		{81, 2},
		{14, 1}
};

uint8_t matrix_6[5][2] = {
		{62, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{38, 1}
};

uint8_t matrix_7[5][2] = {
		{64, 16},
		{64, 8},
		{79, 4},
		{80, 2},
		{96, 1}
};

uint8_t matrix_8[5][2] = {
		{54, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{54, 1}
};

uint8_t matrix_9[5][2] = {
		{50, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{62, 1}
};

uint8_t matrix_A[5][2] = {
		{31, 16},
		{36, 8},
		{68, 4},
		{36, 2},
		{31, 1}
};

uint8_t matrix_B[5][2] = {
		{127, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{54, 1}
};

uint8_t matrix_C[5][2] = {
		{62, 16},
		{65, 8},
		{65, 4},
		{65, 2},
		{34, 1}
};

uint8_t matrix_D[5][2] = {
		{127, 16},
		{65, 8},
		{65, 4},
		{65, 2},
		{62, 1}
};

uint8_t matrix_E[5][2] = {
		{127, 16},
		{73, 8},
		{73, 4},
		{73, 2},
		{73, 1}
};

uint8_t matrix_F[5][2] = {
		{127, 16},
		{72, 8},
		{72, 4},
		{72, 2},
		{72, 1}
};

uint8_t CharMAP[256][5][2];

// other
int index_col = 0;
int bit_ctr = 0;
uint8_t rx_byte;
uint8_t tx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void UART_IR_sendByte(char byte) {
	BaudElapsedFlag = 0;
	HAL_TIM_Base_Start_IT(&htim1);	// start baud rate timer
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // send starting bit '0'
	while (BaudElapsedFlag == 0) {}
	for (int bit_ctr = 0; bit_ctr < 8; bit_ctr++) {
		if ((byte & (0x01<<bit_ctr)) == 0)
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // send 0
		else
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // send 1
		BaudElapsedFlag = 0;
		while(BaudElapsedFlag == 0) {}
	}
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // send stop bit '1'
	while (BaudElapsedFlag == 0) {}
	HAL_TIM_Base_Stop_IT(&htim1);
	BaudElapsedFlag = 0;
}*/

void initCharMAP() {
	for (int i = 0; i < 256; i++) {
		memcpy(CharMAP[i], matrix_QM, sizeof(matrix_QM));
	}
	memcpy(CharMAP[(uint8_t) '0'], matrix_0, sizeof(matrix_0));
	memcpy(CharMAP[(uint8_t) '1'], matrix_1, sizeof(matrix_1));
	memcpy(CharMAP[(uint8_t) '2'], matrix_2, sizeof(matrix_2));
	memcpy(CharMAP[(uint8_t) '3'], matrix_3, sizeof(matrix_3));
	memcpy(CharMAP[(uint8_t) '4'], matrix_4, sizeof(matrix_4));
	memcpy(CharMAP[(uint8_t) '5'], matrix_5, sizeof(matrix_5));
	memcpy(CharMAP[(uint8_t) '6'], matrix_6, sizeof(matrix_6));
	memcpy(CharMAP[(uint8_t) '7'], matrix_7, sizeof(matrix_7));
	memcpy(CharMAP[(uint8_t) '8'], matrix_8, sizeof(matrix_8));
	memcpy(CharMAP[(uint8_t) '9'], matrix_9, sizeof(matrix_9));
	memcpy(CharMAP[(uint8_t) 'A'], matrix_A, sizeof(matrix_A));
	memcpy(CharMAP[(uint8_t) 'B'], matrix_B, sizeof(matrix_B));
	memcpy(CharMAP[(uint8_t) 'C'], matrix_C, sizeof(matrix_C));
	memcpy(CharMAP[(uint8_t) 'D'], matrix_D, sizeof(matrix_D));
	memcpy(CharMAP[(uint8_t) 'E'], matrix_E, sizeof(matrix_E));
	memcpy(CharMAP[(uint8_t) 'F'], matrix_F, sizeof(matrix_F));
}

void showletter(char digit) {
	memcpy(matrix, CharMAP[(uint8_t) digit], sizeof(matrix));

	/* Debug code
	char string[64];
	int length = snprintf(string, sizeof(string), "%d\n", digit);

	HAL_UART_Transmit_DMA(&huart2, string, length);*/
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  initCharMAP();
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, col_activation[col][0]);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, col_activation[col][1]);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, col_activation[col][2]);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, col_activation[col][3]);

  HAL_TIM_Base_Start_IT(&htim3);
  //HAL_TIM_Base_Start_IT(&htim4); // separate timer for led matrix (not necessary)

  HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart1, &rx_byte, 2);
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 34999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2210;
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
  sConfigOC.Pulse = 1105;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8399;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC2 PC3 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// Blocking mode
	/*if (htim->Instance == htim1.Instance) {
		BaudElapsedFlag = 1;
	}*/

	if (htim->Instance == htim1.Instance) {
		if (bit_ctr < 8) {
			if ((tx_byte & (0x01<<bit_ctr)) == 0)
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // send 0
			else
				HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // send 1
			bit_ctr++;
		} else if (bit_ctr == 8) {
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3); // send stop bit '1'
			bit_ctr++;
		} else {
			HAL_TIM_Base_Stop_IT(&htim1);
			bit_ctr = 0;
		}
	}

	if (htim->Instance == TIM3) {
		HAL_SPI_Transmit_DMA(&hspi1, matrix[index_col], 2); // if using only one timer

		GPIO_PinState r[4]; // values read on each row
		r[0] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
		r[1] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		r[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		r[3] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12);
		for (int i = 0; i < 4; i++) { // iterate through rows
			if (r[i] == GPIO_PIN_RESET && prev[col][i] == GPIO_PIN_RESET && prev2[col][i] == GPIO_PIN_SET) { // requires key to be held for 1 full cycle (16ms) before input is processed to prevent bouncing effects
				tx_byte = character[col][i];
				HAL_TIM_Base_Start_IT(&htim1); // start baud rate timer
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // send starting bit '0'

				// Blocking mode
				//UART_IR_sendByte(character[col][i]);
			}
			prev2[col][i] = prev[col][i]; // update previous values matrices
			prev[col][i] = r[i];
		}
		col = (col + 1) % 4; // iterate through columns
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, col_activation[col][0]);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, col_activation[col][1]);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, col_activation[col][2]);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, col_activation[col][3]);
	}

	/*if (htim->Instance == TIM4) {
		HAL_SPI_Transmit_DMA(&hspi1, matrix[index_col], 2);
	}*/
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	index_col = (index_col + 1) % 5; // circularly iterate column
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);		// set and quickly reset RCLK to send data from serial registers to LED matrix
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
 	if (huart == &huart1) {
		HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
		showletter(rx_byte);
	}
}

/*
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart1) {
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, &rx_byte, 1);
		showletter(rx_byte);
	}
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
