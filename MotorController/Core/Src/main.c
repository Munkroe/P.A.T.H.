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

#include "circle_queue_Vector3.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RIGHT_MOTOR_CHANNEL TIM_CHANNEL_1
#define LEFT_MOTOR_CHANNEL TIM_CHANNEL_2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
char uart_rxbuffer[UART_IN_BUF_SIZE] = { 0 };
UartCommHandler rxHandler;

char uart_txbuffer[UART_IN_BUF_SIZE] = { 0 };
UartCommHandler txHandler;

float batteryVoltage = 0.0;
float voltageMeasScaling = (3.47 / (4096)) * (1 + 2.63); // Voltage divider ratio - Reference voltage was found experimentally
bool control_cmds_en = false;

float spamCheckY = 0.;
float spamCheckX = 0.;

float spamCheckPhi = 0.0;
char packedMotorData[50] = { 0 };

float posX = 0.0;
float posXPrev = 0.0;
float posY = 0.0;
float posYPrev = 0.0;
float posPhi = 0.0;
float posPhiPrev = 0.0;
float velX = 0.0;
float velY = 0.0;
float velocity = 0.0;
float velPhi = 0.0;
uint8_t position[24] = { 0 };

extern float orientAngle;

uint8_t MOTORID = 2;

float positionCalculationPeriod;
float controllerPeriod;

// IMU variables
Vector3Queue *measQueue;

typedef struct {
	uint32_t time;
	float speed;
	Vector3 entry;
} IMU_MeasEntry;

uint32_t imu_meas_index = 0;
IMU_MeasEntry imu_meas_arr[IMU_MEAS_AMOUNT] = { 0 };

extern void (*accel_capt_callb)(Vector3*), (*gyro_capt_callb)(Vector3*);
void (**imu_capture_callb)(Vector3*);

// Motor structs

Motor motorR;
MotorEncoder encoderR;
MotorController controllerR;

Motor motorL;
MotorEncoder encoderL;
MotorController controllerL;

uint32_t ticks = 0;
uint32_t ticks_ms = 0;
uint32_t laps_c = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

float LP_filter(float x_old2, float x_old1, float x, float y_old3, float y_old2,
		float y_old1);
int8_t samples_equivalence_test(float *a, float *b, uint32_t len,
		float tolerance);
int8_t IMU_LP_Filter_test(double signal_freq, double sample_freq,
		float *comp_seq, Vector3Queue *rawQueue, Vector3Queue *filtQueue);

void packThe6Floats();
void uart_in_handle(char*, uint32_t, uint8_t);
int8_t uart_in_handle_reset(char*, uint32_t);
int8_t uart_in_handle_reference(char*, uint32_t);
int8_t uart_transmit_VectorXY(uint8_t frameid, Vector3 data);
int8_t InitialCalibration();
void MainLoop();
void IMU_StartMeasurements(void (**capture_callb)(Vector3*), bool rotate,
		float movePeriod, float wheelSpeed, uint8_t moveRepetitions);
void MainLoopWaitMicros(uint32_t wait_us);
void MainLoopWaitMillis(uint32_t wait_ms);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
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

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM6_Init();
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM7_Init();
	MX_TIM2_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */

	positionCalculationPeriod = ((htim6.Instance->ARR + 1)
			/ (80000000.0 / (htim6.Instance->PSC + 1)));
	controllerPeriod = ((htim2.Instance->ARR + 1)
			/ (80000000.0 / (htim2.Instance->PSC + 1)));

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

	uart_init();
	IMU_init();

	reset_odometry();

	if (!micros_init(&htim2, HAL_RCC_GetPCLK1Freq()))
		Error_Handler();

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	InitialCalibration();


	float freq = 1;
	while (1) {


		IMU_StartMeasurements(&accel_capt_callb, true, 1/freq, 30, 4);

		MainLoop();

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
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
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */
	__HAL_RCC_ADC_CLK_ENABLE();
	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00702991;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 2000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 80 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10000 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_OC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_TIMING;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 160 - 1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 50000 - 1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM7_Init(void) {

	/* USER CODE BEGIN TIM7_Init 0 */

	/* USER CODE END TIM7_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM7_Init 1 */

	/* USER CODE END TIM7_Init 1 */
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 65535;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 12207 - 1;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 500000;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, DIR_L1_Pin | testLED_Pin | DIR_L2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DIR_R1_Pin | DIR_R2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Motor_counterclock_right_Pin Motor_Left_clock_Pin orientation_counterclock_Pin */
	GPIO_InitStruct.Pin = Motor_counterclock_right_Pin | Motor_Left_clock_Pin
			| orientation_counterclock_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_L1_Pin testLED_Pin DIR_L2_Pin */
	GPIO_InitStruct.Pin = DIR_L1_Pin | testLED_Pin | DIR_L2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : motor_Right_clock_Pin Motor_left_counterclock_Pin orientation_clock_Pin */
	GPIO_InitStruct.Pin = motor_Right_clock_Pin | Motor_left_counterclock_Pin
			| orientation_clock_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DIR_R1_Pin DIR_R2_Pin */
	GPIO_InitStruct.Pin = DIR_R1_Pin | DIR_R2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void reset_odometry() {
	// Motor Initialization
	motor_init(&motorR, 'R');
	motor_init(&motorL, 'L');

	motorEncoder_init(&encoderR);
	motorEncoder_init(&encoderL);

	motorController_init(&controllerR, &motorR, &encoderR);
	motorController_init(&controllerL, &motorL, &encoderL);

	orientation_reset();
}

void motor_init(Motor *m, char name) {
	m->name = name;
	m->direction = 0;
	m->dutyCycle = 0.0;
}

void motorEncoder_init(MotorEncoder *e) {
	e->fineAdjustment = 0;
	e->lastAngle = 0.0;
	e->lastTicks = 0;
	e->output = 0.0;
	e->revolutions = 0;
}

void motorController_init(MotorController *c, Motor *m, MotorEncoder *e) {
	c->Encoder = e;
	c->lastError = 0.0;
	c->motor = m;
	c->reference = 0.0;
	c->controlVoltage = 0.0;
	c->measAngVel = 0.0;
}

void uart_init() {

	// Initialize RX
	rxHandler.direction = DIRECTION_RX;
	rxHandler.huart = &huart2;
	rxHandler.rxHandleFunc = &uart_in_handle;
	rxHandler.buffer = uart_rxbuffer;
	rxHandler.bufferSize = sizeof(uart_rxbuffer);
	rxHandler.frameid_enabled = false;

	if (!uart_init_rx(&rxHandler))
		Error_Handler();

	// Initialize TX
	txHandler.direction = DIRECTION_TX;
	txHandler.huart = &huart2;
	txHandler.buffer = uart_txbuffer;
	txHandler.bufferSize = sizeof(uart_txbuffer);
	txHandler.frameid_enabled = true;

	if (!uart_init_tx(&txHandler))
		Error_Handler();
}

void uart_in_handle(char *uart_msg, uint32_t len, uint8_t id) {

	if (uart_in_handle_reset(uart_msg, len))
		return;
	if (control_cmds_en) {
		if (uart_in_handle_reference(uart_msg, len))
			return;
	}

}

int8_t uart_in_handle_reset(char *uart_msg, uint32_t len) {
	if (strcmp(uart_msg, "reset") == 0) {

		reset_odometry();
		sendPositionAndVelocity();

		return 1;
	}
	return 0;
}
int8_t uart_in_handle_reference(char *uart_msg, uint32_t len) {

	// Check length of msg
	if (len != 10)
		return 0;

	// Retrieve reference for right wheel
	if (uart_msg[0] == 'R') {
		memcpy(&controllerR.reference, uart_msg + 1, 4);

	} else
		return 0;

	// Retrieve reference for left wheel
	if (uart_msg[5] == 'L') {
		memcpy(&controllerL.reference, uart_msg + 6, 4);
	} else
		return 0;

	return 1;
}

int8_t uart_transmit_VectorXY(uint8_t frameid, Vector3 data) {
	char msg[8] = { 0 };

	memcpy(msg, &(data.x), sizeof(float));
	memcpy(msg + sizeof(float), &(data.y), sizeof(float));

	return uart_transmit(&txHandler, msg, sizeof(msg), frameid);
}

float calcDistance(MotorController *c) {
	float deltaTicks = c->Encoder->output * TOTAL_WHEEL_TICKS
			- c->Encoder->lastTicks;
	return M_PI * WHEELDIA * (deltaTicks / TOTAL_WHEEL_TICKS);
}

void calcPositionAndVelocity() {
	float distR = calcDistance(&controllerR);
	float distL = calcDistance(&controllerL);
	controllerR.Encoder->lastTicks = controllerR.Encoder->output
			* TOTAL_WHEEL_TICKS;
	controllerL.Encoder->lastTicks = controllerL.Encoder->output
			* TOTAL_WHEEL_TICKS;
	float deltaDistance = (distL + distR) / 2;
	posX = posX + deltaDistance * cos(posPhi);
	posY = posY + deltaDistance * sin(posPhi);
	posPhi = posPhi + (distR - distL) / DISBETWHEEL;

	velPhi = ((posPhi - posPhiPrev)) / positionCalculationPeriod;
	velX = (posX - posXPrev) / positionCalculationPeriod;
	velY = (posY - posYPrev) / positionCalculationPeriod;
	velocity = deltaDistance / positionCalculationPeriod;

	posPhiPrev = posPhi;
	posXPrev = posX;
	posYPrev = posY;
}

void updatePositionsAndVelocities() {

	// Position and velocity data from wheel encoders
	calcPositionAndVelocity();

	if (spamCheckX != posX || spamCheckY != posY || spamCheckPhi != posPhi) {
		spamCheckX = posX;
		spamCheckY = posY;
		spamCheckPhi = posPhi;
		sendPositionAndVelocity();
	}

	// Encoder data from top plate
	calcOrientOutput();
	sendOrientData();

	IMU_TransmitData(&txHandler);
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
	pointer = &velX; //&controllerL.Encoder->output;
	for (int m = 12; m < 16; m++) {
		position[m] = *(pointer + m - 12);
	}
	pointer = &velY; //&controllerR.Encoder->output;
	for (int n = 16; n < 20; n++) {
		position[n] = *(pointer + n - 16);
	}
	pointer = &velPhi;
	for (int o = 20; o < 24; o++) {
		position[o] = *(pointer + o - 20);
	}
}

void sendPositionAndVelocity() {
	packThe6Floats();
	uart_transmit(&txHandler, position, sizeof(position), UART_ID_MOTOR);
	memset(position, 0, sizeof(position));
}

void resetEncoder(MotorController *c) {
	c->Encoder->output = 0.0;
	c->Encoder->fineAdjustment = 0;
	c->Encoder->revolutions = 0;
}

void clockcheckRight() {
	if (HAL_GPIO_ReadPin(motor_Right_clock_GPIO_Port, motor_Right_clock_Pin)
			== HAL_GPIO_ReadPin(Motor_counterclock_right_GPIO_Port,
			Motor_counterclock_right_Pin)) {
		controllerR.motor->direction = -1;
		controllerR.Encoder->fineAdjustment = abs(
				(controllerR.Encoder->fineAdjustment
						+ controllerR.motor->direction) % TOTAL_WHEEL_TICKS);
	} else {
		controllerR.motor->direction = 1;
		controllerR.Encoder->fineAdjustment = abs(
				(controllerR.Encoder->fineAdjustment
						+ controllerR.motor->direction)
						% (TOTAL_WHEEL_TICKS + 1));
	}

	checkRevolutions(&controllerR);
}

void counterclockcheckRight() {
	if (HAL_GPIO_ReadPin(motor_Right_clock_GPIO_Port, motor_Right_clock_Pin)
			== HAL_GPIO_ReadPin(Motor_counterclock_right_GPIO_Port,
			Motor_counterclock_right_Pin)) {
		controllerR.motor->direction = 1;
		controllerR.Encoder->fineAdjustment = abs(
				(controllerR.Encoder->fineAdjustment
						+ controllerR.motor->direction)
						% (TOTAL_WHEEL_TICKS + 1));
	} else {
		controllerR.motor->direction = -1;
		controllerR.Encoder->fineAdjustment = abs(
				(controllerR.Encoder->fineAdjustment
						+ controllerR.motor->direction) % TOTAL_WHEEL_TICKS);
	}

	checkRevolutions(&controllerR);
}

void clockcheckLeft() {
	if (HAL_GPIO_ReadPin(Motor_Left_clock_GPIO_Port, Motor_Left_clock_Pin)
			== HAL_GPIO_ReadPin(Motor_left_counterclock_GPIO_Port,
			Motor_left_counterclock_Pin)) {
		controllerL.motor->direction = -1;
		controllerL.Encoder->fineAdjustment = abs(
				(controllerL.Encoder->fineAdjustment
						+ controllerL.motor->direction) % TOTAL_WHEEL_TICKS);
	} else {
		controllerL.motor->direction = 1;
		controllerL.Encoder->fineAdjustment = abs(
				(controllerL.Encoder->fineAdjustment
						+ controllerL.motor->direction)
						% (TOTAL_WHEEL_TICKS + 1));
	}

	checkRevolutions(&controllerL);
}

void counterclockcheckLeft() {
	if (HAL_GPIO_ReadPin(Motor_Left_clock_GPIO_Port, Motor_Left_clock_Pin)
			== HAL_GPIO_ReadPin(Motor_left_counterclock_GPIO_Port,
			Motor_left_counterclock_Pin)) {
		controllerL.motor->direction = 1;
		controllerL.Encoder->fineAdjustment = abs(
				(controllerL.Encoder->fineAdjustment
						+ controllerL.motor->direction)
						% (TOTAL_WHEEL_TICKS + 1));
	} else {
		controllerL.motor->direction = -1;
		controllerL.Encoder->fineAdjustment = abs(
				(controllerL.Encoder->fineAdjustment
						+ controllerL.motor->direction) % TOTAL_WHEEL_TICKS);
	}

	checkRevolutions(&controllerL);
}

void checkRevolutions(MotorController *c) {
	if (c->Encoder->fineAdjustment == 0 && c->motor->direction == 1) {
		c->Encoder->revolutions = c->Encoder->revolutions + c->motor->direction;
		c->Encoder->fineAdjustment = 1;
	} else if (c->Encoder->fineAdjustment == 0 && c->motor->direction == -1) {
		c->Encoder->revolutions = c->Encoder->revolutions + c->motor->direction;
		c->Encoder->fineAdjustment = TOTAL_WHEEL_TICKS;
	}

	/*if (wheel = 'L') {
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
	 }*/
}

void calcOutput(MotorEncoder *e) {
	e->output = e->revolutions
			+ ((float) e->fineAdjustment / TOTAL_WHEEL_TICKS);
}

void calculateError(MotorController *c) {
	c->lastError = c->reference - c->measAngVel;
}

void nextVoltage_newImp(MotorController *c) {
	// *********************************************************************
	// ********* THIS CONTROLLER IMPLEMENTAITON IS NOT FULLY WORKING *******
	// *********************************************************************
	// Kill motors if we want to stop and are practically already stopped
	if (c->reference == 0.0 && abs(c->measAngVel < MOTOR_VELOCITY_DEADZONE))
		c->controlVoltage = 0.0;
	else
		// Normal controller operation
		c->controlVoltage = c->lastError * 2.82 * controllerPeriod
				+ c->controlVoltage;

	if (c->controlVoltage > MOTOR_VOLTAGE_MAX - MOTOR_VOLTAGE_OFFSET) {
		c->controlVoltage = MOTOR_VOLTAGE_MAX - MOTOR_VOLTAGE_OFFSET;
	} else if (c->controlVoltage < -MOTOR_VOLTAGE_MAX + MOTOR_VOLTAGE_OFFSET) {
		c->controlVoltage = -MOTOR_VOLTAGE_MAX + MOTOR_VOLTAGE_OFFSET;
	}

	/* Change direction only if we both want to actually
	 * change direction (reference), and output voltage is at the offset edge
	 * (controlVoltage ~= 0 ==> driveVoltage = MOTOR_VOLTAGE_OFFSET).
	 * This enables the controlVoltage to swing around 0V,
	 * without the MOTOR_VOLTAGE_OFFSET frantically changing polarity.
	 */
	if (/*c->reference > 0 &&*/c->controlVoltage >= 0) {
		c->motor->direction = 1;
	} else if (/*c->reference < 0 &&*/c->controlVoltage <= 0) {
		c->motor->direction = -1;
	}

	if (c->controlVoltage != 0.0)
		c->driveVoltage = MOTOR_VOLTAGE_OFFSET * c->motor->direction
				+ c->controlVoltage;
	else
		c->driveVoltage = 0.0;
	// *********************************************************************
}

void nextVoltage(MotorController *c) {

	// Old implementation
	c->driveVoltage = c->lastError * 2.82 * controllerPeriod + c->driveVoltage;
	if (c->driveVoltage > MOTOR_VOLTAGE_MAX) {
		c->driveVoltage = MOTOR_VOLTAGE_MAX;
	} else if (c->driveVoltage < -MOTOR_VOLTAGE_MAX) {
		c->driveVoltage = -MOTOR_VOLTAGE_MAX;
	}
}

void updateAngularVelocity(MotorController *c) {
	float deltaAngle = c->Encoder->output * 2 * M_PI - c->Encoder->lastAngle;
	c->Encoder->lastAngle = c->Encoder->output * 2 * M_PI;
	c->measAngVel = deltaAngle / controllerPeriod;

	float angVelPos = c->measAngVel;
	if (angVelPos < 0)
		angVelPos = -angVelPos;
	if (angVelPos < MOTOR_ANGULAR_VELOCITY_MIN) {
		c->measAngVel = 0;
	}
}

void updateDutyCycle_newImp(MotorController *c) {
	// *********************************************************************
	// ********* THIS CONTROLLER IMPLEMENTAITON IS NOT FULLY WORKING *******
	// *********************************************************************
	if (c->driveVoltage == 0.0) {
		c->motor->dutyCycle = 0;
		return;
	}
	// *********************************************************************

	float pwm = c->driveVoltage / batteryVoltage;

	if (pwm < 0)
		pwm = -pwm;
	if (pwm > 1.0)
		pwm = 1.0;
	c->motor->dutyCycle = pwm;
}

void updateDutyCycle(MotorController *c) {

	// *** Old controller implementation
	if (c->driveVoltage > 0) {
		c->motor->direction = 1;
	} else if (c->driveVoltage < 0) {
		c->motor->direction = -1;
	} else {
		c->motor->dutyCycle = 0;
		return;
	}
	// ***

	float pwm = c->driveVoltage / batteryVoltage;

	if (pwm < 0)
		pwm = -pwm;
	if (pwm > 1.0)
		pwm = 1.0;
	c->motor->dutyCycle = pwm;
}

void setDutyCycle_newImp(MotorController *c) {
	if (c->motor->name == 'R') {
		htim1.Instance->CCR1 = (uint32_t) ((htim1.Instance->ARR)
				* c->motor->dutyCycle);

		if (c->motor->direction == 1) {
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 1);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 0);
		} else if (c->motor->direction == -1) {
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 0);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 1);
		} else {
			// MOTOR STOP
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 0);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 0);
		}
	} else if (c->motor->name == 'L') {
		htim1.Instance->CCR2 = (uint32_t) ((htim1.Instance->ARR)
				* c->motor->dutyCycle);

		if (c->motor->direction == 1) {
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, 1);
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, 0);
		} else if (c->motor->direction == -1) {
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, 0);
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, 1);
		} else {
			// MOTOR STOP
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 0);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 0);
		}
	} else {
		return;
	}
}

void setDutyCycle(MotorController *c) {
	if (c->motor->name == 'R') {
		htim1.Instance->CCR1 = (uint32_t) ((htim1.Instance->ARR)
				* c->motor->dutyCycle);

		if (c->motor->direction == 1) {
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 1);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 0);
		} else if (c->motor->direction == -1) {
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 0);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 1);
		} else {
			// MOTOR STOP
			//HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 0);
			//HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 0);
		}
	} else if (c->motor->name == 'L') {
		htim1.Instance->CCR2 = (uint32_t) ((htim1.Instance->ARR)
				* c->motor->dutyCycle);

		if (c->motor->direction == 1) {
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, 1);
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, 0);
		} else if (c->motor->direction == -1) {
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, 0);
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, 1);
		} else {
			// MOTOR STOP
			//HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, 0);
			//HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, 0);
		}
	} else {
		return;
	}
}

void controller(MotorController *c) {
	calcOutput(c->Encoder);

	// Measure the angular velocity (feedback)
	updateAngularVelocity(c);

	// Calculate next controlVoltage according to the controller design
	//nextVoltage_newImp(c);
	nextVoltage(c);

	// Calculate current error to use for next iteration
	calculateError(c);

	// Update the duty cycle
	//updateDutyCycle_newImp(c);
	updateDutyCycle(c);

	//setDutyCycle_newImp(c);
	setDutyCycle(c);
}

void controlBothMotors() {
	uint32_t micros_start = micros();
	uart_rxhandle(&rxHandler);
	controller(&controllerR);
	controller(&controllerL);
	uint32_t micros_end = micros();
	uint32_t deltaT = micros_end - micros_start;
}

void UpdateBatteryVoltage() {
	HAL_ADC_Start(&hadc1); // Start ADC conversion
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Wait for conversion to complete
	uint32_t adc_val = HAL_ADC_GetValue(&hadc1); // Get the ADC value
	batteryVoltage = adc_val * voltageMeasScaling;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	IMU_TransmitComplete();

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	IMU_ReceiveComplete();
}

void Callback_1Hz() {

}

void Callback_1kHz() {
	// Set the IMU read pointer to the sensor data registers
	IMU_RequestData();
}

void MainLoop() {
	// Retrieve data if an IMU data request has just been issued
	IMU_RetrieveData();
	// Handle and process data if data retrieval has been requested and we have just received data
	IMU_HandleReceivedData();
}

void MainLoopWaitMicros(uint32_t wait_us) {
	uint32_t startTime = micros();

	while (micros() < startTime + wait_us) {
		MainLoop();
	}
}

void MainLoopWaitMillis(uint32_t wait_ms) {
	uint32_t startTime = HAL_GetTick();

	while (HAL_GetTick() < startTime + wait_ms) {
		MainLoop();
	}
}

void StopMovement() {
	controllerL.reference = 0.0;
	controllerR.reference = 0.0;

	while (controllerL.measAngVel != 0.0 || controllerR.measAngVel != 0.0) {
		MainLoop();
	}
}

int8_t InitialCalibration() {
	control_cmds_en = false;

	uint32_t start_time = HAL_GetTick();

	// Wait till while the gyro settles
	while (HAL_GetTick() < start_time + IMU_CALIBRATION_TIME_MS) {
		MainLoop();
	}

	IMU_CalculateOffsets();

	control_cmds_en = true;
}

void IMU_Measurements_CaptureEntry(Vector3 *entry) {
	IMU_MeasEntry m = { .time = micros(), .entry = *entry, .speed = (imu_capture_callb == &gyro_capt_callb ? velPhi : velocity) };

	if (imu_meas_index < IMU_MEAS_AMOUNT)
		imu_meas_arr[imu_meas_index] = m;

	if (++imu_meas_index == IMU_MEAS_AMOUNT) {
		// Measurements done
		*imu_capture_callb = NULL;
		imu_capture_callb = NULL;
	}
}

void IMU_StartMeasurements(void (**capture_callb)(Vector3*), bool rotate,
		float movePeriod, float wheelSpeed, uint8_t moveRepetitions) {

	// Reset data array
	imu_meas_index = 0;
	memset(imu_meas_arr, 0, IMU_MEAS_AMOUNT * sizeof(IMU_MeasEntry));

	imu_capture_callb = capture_callb;

	// Disable external motor control
	control_cmds_en = false;

	StopMovement();

	// Start capturing data
	*imu_capture_callb = &IMU_Measurements_CaptureEntry;

	// Wait a moment
	MainLoopWaitMillis(IMU_MEAS_WAIT_TIME);

	// Do some erratic movement
	float sign = 1.0;
	uint32_t period_us = (uint32_t) ((movePeriod / 2.0) * 1000000);
	uint32_t endTime = micros() + period_us;
	for (uint8_t i = 0; i < moveRepetitions; i++) {
		controllerL.reference = wheelSpeed * sign;
		controllerR.reference = wheelSpeed * sign * (rotate ? -1.0 : 1.0);

		while (micros() < endTime) {
			MainLoop();
		}
		endTime += period_us;

		sign = -sign;
	}

	StopMovement();

	// Enable external motor control
	control_cmds_en = true;
}

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

