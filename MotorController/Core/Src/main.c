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
#include "DCMotorDriver.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RIGHT_MOTOR_CHANNEL TIM_CHANNEL_1
#define LEFT_MOTOR_CHANNEL TIM_CHANNEL_2
#define WHEELDIA 80
#define DISBETWHEEL 380
#define TOTAL_WHEEL_TICKS 1920

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t message = 0;

float T = 0.01;
float refR = 0.0;
float refL = 0.0;
float lastErrorR = 0.0;
float lastErrorL = 0.0;
float voltageR = 0.0;
float voltageL = 0.0;
float batteryVoltage = 0.0;
float voltageMeasScaling = (3.47/(4096))*(1+2.63); // Voltage divider ratio - Reference voltage was found experimentally


int16_t fineadjustmentLeft = 0;
int32_t revolutionLeft = 0;
int8_t directionLeft = 0;
float spamCheckY = 0.;
float spamCheckPhi = 0.0;
char packedMotorData[50] = { 0 };

int16_t fineadjustmentRight = 0;
int32_t revolutionRight = 0;
int8_t directionRight = 0;
float spamCheckX = 0.;
float tempTicksR = 0.0;
float tempTicksL = 0.0;
float tempRadsR = 0.0;
float tempRadsL = 0.0;
float posX = 0.0;
float posXPrev = 0.0;
float posY = 0.0;
float posYPrev = 0.0;
float posPhi = 0.0;
float posPhiPrev = 0.0;
float velX = 0.0;
float velY = 0.0;
float velPhi = 0.0;
uint8_t position[25] = { 0 };

float outputLeft = 0.0;
float outputRight = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
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
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_UART_Receive_IT(&huart2, &message, 1);
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);


	motor_init(&htim1, RIGHT_MOTOR_CHANNEL, LEFT_MOTOR_CHANNEL);
	motor_start('R');
	motor_setDirection('R', 0);
	motor_setPWM('R', 0.5);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

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
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  __HAL_RCC_ADC_CLK_ENABLE();
  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 50000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 65535;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 12207-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_L1_Pin|DIR_L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_R1_Pin|DIR_R2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Motor_Left_clock_Pin */
  GPIO_InitStruct.Pin = Motor_Left_clock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Motor_Left_clock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_L1_Pin DIR_L2_Pin */
  GPIO_InitStruct.Pin = DIR_L1_Pin|DIR_L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : motor_Right_clock_Pin Motor_left_counterclock_Pin Motor_counterclock_right_Pin */
  GPIO_InitStruct.Pin = motor_Right_clock_Pin|Motor_left_counterclock_Pin|Motor_counterclock_right_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_R1_Pin DIR_R2_Pin */
  GPIO_InitStruct.Pin = DIR_R1_Pin|DIR_R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void calcPositionAndVelocity() {
	float deltaTicksR = outputRight * TOTAL_WHEEL_TICKS - tempTicksR;
	float D_r = M_PI * WHEELDIA * (deltaTicksR / TOTAL_WHEEL_TICKS);
	float deltaTicksL = outputLeft * TOTAL_WHEEL_TICKS - tempTicksL;
	float D_l = M_PI * WHEELDIA * (deltaTicksL / TOTAL_WHEEL_TICKS);
	tempTicksR = outputRight * TOTAL_WHEEL_TICKS;
	tempTicksL = outputLeft * TOTAL_WHEEL_TICKS;
	float D_c = (D_l + D_r) / 2;
	posX = posX + D_c * cos(posPhi);
	posY = posY + D_c * sin(posPhi);
	posPhi = posPhi + (D_r - D_l) / DISBETWHEEL;
	//Vi ved, at der går rundt regnet 5 ms mellem vi kommer her:
	velPhi = ((posPhi - posPhiPrev)) / 0.005;
	velX = (posX - posXPrev) / 0.005;
	velY = (posY - posYPrev) / 0.005;
	//OBS. Tiden, koden over dette stykke tager, gør denne udregning forkert, men møj tæt på.
	posPhiPrev = posPhi;
	posXPrev = posX;
	posYPrev = posY;
}

void packThe6Floats() {
	uint8_t *pointer = &posX;
	for (int i = 0; i < 4; i++) {
		position[i] = *(pointer + i);
	}

	pointer = &posY;
	for (int k = 4; k < 8; k++) {
		position[k] = *(pointer + k - 4);
	}
	pointer = &posPhi;
	for (int j = 8; j < 12; j++) {
		position[j] = *(pointer + j - 8);
	}
	pointer = &velX;
	for (int m = 12; m < 16; m++) {
		position[m] = *(pointer + m - 12);
	}
	pointer = &velY;
	for (int n = 16; n < 20; n++) {
		position[n] = *(pointer + n - 16);
	}
	pointer = &velPhi;
	for (int o = 20; o < 24; o++) {
		position[o] = *(pointer + o - 20);
	}
}

void sendData() {
	if (spamCheckX != posX || spamCheckY != posY || spamCheckPhi != posPhi) {
		spamCheckX = posX;
		spamCheckY = posY;
		spamCheckPhi = posPhi;
		packThe6Floats();
		memset(packedMotorData, 0, sizeof(packedMotorData));

		to_frame(packedMotorData, position, 3);
		HAL_UART_Transmit(&huart2, packedMotorData, sizeof(packedMotorData), HAL_MAX_DELAY);
	}
}

void resetEncoder() {

	if (message == 76) //'L' - left motor
			{
		outputLeft = 0.;
		fineadjustmentLeft = 0;
		revolutionLeft = 0.;
	}
	if (message == 82) //'R' - right motor
			{
		outputRight = 0.0;
		fineadjustmentRight = 0.0;
		revolutionRight = 0.0;
	}

}

void clockcheckRight() {
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {
		directionRight = -1;
		fineadjustmentRight = abs((fineadjustmentRight + directionRight) % TOTAL_WHEEL_TICKS);
	} else {
		directionRight = 1;
		fineadjustmentRight = abs((fineadjustmentRight + directionRight) % 1921);
	}

	checkRevolutions('R');
}

void counterclockcheckRight() {
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) {
		directionRight = 1;
		fineadjustmentRight = abs((fineadjustmentRight + directionRight) % 1921);
	} else {
		directionRight = -1;
		fineadjustmentRight = abs((fineadjustmentRight + directionRight) % TOTAL_WHEEL_TICKS);
	}

	checkRevolutions('R');
}

void clockcheckLeft() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) {
		directionLeft = -1;
		fineadjustmentLeft = abs((fineadjustmentLeft + directionLeft) % TOTAL_WHEEL_TICKS);
	} else {
		directionLeft = 1;
		fineadjustmentLeft = abs((fineadjustmentLeft + directionLeft) % 1921);
	}

	checkRevolutions('L');
}

void counterclockcheckLeft() {
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)) {
		directionLeft = 1;
		fineadjustmentLeft = abs((fineadjustmentLeft + directionLeft) % 1921);
	} else {
		directionLeft = -1;
		fineadjustmentLeft = abs((fineadjustmentLeft + directionLeft) % TOTAL_WHEEL_TICKS);
	}

	checkRevolutions('L');
}

void checkRevolutions(char wheel) {
	if (wheel = 'L') {
		if (fineadjustmentLeft == 0 && directionLeft == 1) {
			revolutionLeft = revolutionLeft + directionLeft;
			fineadjustmentLeft = 1;
		} else if (fineadjustmentLeft == 0 && directionLeft == -1) {
			revolutionLeft = revolutionLeft + directionLeft;
			fineadjustmentLeft = TOTAL_WHEEL_TICKS;
		}
	}
	if (wheel = 'R')
		if (fineadjustmentRight == 0 && directionRight == 1) {
			revolutionRight = revolutionRight + directionRight;
			fineadjustmentRight = 1;
		} else if (fineadjustmentRight == 0 && directionRight == -1) {
			revolutionRight = revolutionRight + directionRight;
			fineadjustmentRight = TOTAL_WHEEL_TICKS;
		}
}

void calcOutput() {
	outputLeft = revolutionLeft + (fineadjustmentLeft / TOTAL_WHEEL_TICKS);
	outputRight = revolutionRight + (fineadjustmentRight / TOTAL_WHEEL_TICKS);
}

float calculateError(float ref, float meas) {
	return ref - meas;
}

float nextVoltage(float lastErr, float lastVoltage) {
	return lastErr * 2.82 * T + lastVoltage;
}

void controller() {
	calcOutput();

	// Measure the angular velocity (feedback)
	float deltaRadsR = outputRight * 2 * M_PI - tempRadsR;
	float deltaRadsL = outputLeft * 2 * M_PI - tempRadsL;
	tempRadsR = outputRight * 2 * M_PI;
	tempRadsL = outputLeft * 2 * M_PI;
	float angularVelR = deltaRadsR / T;
	float angularVelL = deltaRadsL / T;

	// Calculate next voltage according to the controller design
	voltageR = nextVoltage(lastErrorR, voltageR);
	voltageL = nextVoltage(lastErrorL, voltageL);

	// Calculate current error to use for next iteration
	lastErrorR = calculateError(refR, angularVelR);
	lastErrorL = calculateError(refL, angularVelL);

	float pwmR = voltageR / batteryVoltage;
	float pwmL = voltageL / batteryVoltage;
	if (pwmR > 1.0) pwmR = 1.0;
	if (pwmL > 1.0) pwmL = 1.0;

	motor_setPWM('R', pwmR);
	motor_setPWM('L', pwmL);
}

void UpdateBatteryVoltage() {
	HAL_ADC_Start(&hadc1); // Start ADC conversion
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversion to complete
	uint32_t adc_val = HAL_ADC_GetValue(&hadc1); // Get the ADC value
	batteryVoltage = adc_val*voltageMeasScaling;
}

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
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/