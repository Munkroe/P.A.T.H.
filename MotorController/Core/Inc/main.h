/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Vector3.h"
#include "stdbool.h"
#include "math.h"
#include "microsecond_counter.h"
#include "orientation.h"
#include "frame_comm.h"
#include "IMU_handler.h"
#include "IMU_filter.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef struct MotorEncoder {
	int32_t lastTicks; 		// Previously
	float lastAngle;
	int16_t fineAdjustment;
	int32_t revolutions;
	float output;
} MotorEncoder;

typedef struct Motor {
	char name; // 'R' or 'L'
	int8_t direction; // -1 or 1
	float dutyCycle;
} Motor;

typedef struct MotorController {
	float reference;
	float measAngVel;
	float lastError;

	float controlVoltage;
	float driveVoltage;
	Motor *motor;
	MotorEncoder *Encoder;
} MotorController;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void UpdateBatteryVoltage();
void reset_odometry();
void Callback_1Hz();
void Callback_1kHz();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BatteryVoltage_Pin GPIO_PIN_3
#define BatteryVoltage_GPIO_Port GPIOA
#define Motor_counterclock_right_Pin GPIO_PIN_4
#define Motor_counterclock_right_GPIO_Port GPIOA
#define Motor_counterclock_right_EXTI_IRQn EXTI4_IRQn
#define Motor_Left_clock_Pin GPIO_PIN_5
#define Motor_Left_clock_GPIO_Port GPIOA
#define Motor_Left_clock_EXTI_IRQn EXTI9_5_IRQn
#define DIR_L1_Pin GPIO_PIN_6
#define DIR_L1_GPIO_Port GPIOA
#define motor_Right_clock_Pin GPIO_PIN_0
#define motor_Right_clock_GPIO_Port GPIOB
#define motor_Right_clock_EXTI_IRQn EXTI0_IRQn
#define Motor_left_counterclock_Pin GPIO_PIN_1
#define Motor_left_counterclock_GPIO_Port GPIOB
#define Motor_left_counterclock_EXTI_IRQn EXTI1_IRQn
#define PWM_R_Pin GPIO_PIN_8
#define PWM_R_GPIO_Port GPIOA
#define PWM_L_Pin GPIO_PIN_9
#define PWM_L_GPIO_Port GPIOA
#define testLED_Pin GPIO_PIN_10
#define testLED_GPIO_Port GPIOA
#define orientation_counterclock_Pin GPIO_PIN_11
#define orientation_counterclock_GPIO_Port GPIOA
#define orientation_counterclock_EXTI_IRQn EXTI15_10_IRQn
#define DIR_L2_Pin GPIO_PIN_12
#define DIR_L2_GPIO_Port GPIOA
#define orientation_clock_Pin GPIO_PIN_3
#define orientation_clock_GPIO_Port GPIOB
#define orientation_clock_EXTI_IRQn EXTI3_IRQn
#define DIR_R1_Pin GPIO_PIN_6
#define DIR_R1_GPIO_Port GPIOB
#define DIR_R2_Pin GPIO_PIN_7
#define DIR_R2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define WHEELDIA 0.086
#define DISBETWHEEL 0.3637
#define TOTAL_WHEEL_TICKS 1920
#define UART_IN_BUF_SIZE 256
#define MOTOR_VOLTAGE_MAX 13.0
#define MOTOR_VOLTAGE_STALL 2.5
#define MOTOR_ANGULAR_VELOCITY_MIN 0.001
#define MOTOR_VOLTAGE_OFFSET 0 // 5
#define MOTOR_VELOCITY_DEADZONE 0.01
#define UART_ID_MOTOR 2
#define UART_ID_ORIENTATION 3
#define UART_ID_ACCELGYRO 4
#define IMU_CALIBRATION_TIME_MS 250
#define IMU_MEAS_AMOUNT 2000
#define IMU_MEAS_WAIT_TIME 100 // ms
#define IMU_MEAS_MOVE_SPEED 30.0 // m/s

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
