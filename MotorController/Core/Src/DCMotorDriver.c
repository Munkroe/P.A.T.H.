#include "DCMotorDriver.h"

TIM_HandleTypeDef *htim;
uint32_t right_channel = 0;
uint32_t left_channel = 0;

void motor_initi(TIM_HandleTypeDef *htimer, uint32_t r_channel, uint32_t l_channel) {
	htim = htimer;
	right_channel = r_channel;
	left_channel = l_channel;
}

void motor_setPWM(char wheel, float dutycycle) {
	if (dutycycle > 1.0f)
		dutycycle = 1.0f;
	if (wheel == 'R') {
		htim->Instance->CCR1 = (uint32_t) ((htim->Instance->ARR) * dutycycle);
	} else if (wheel == 'L') {
		htim->Instance->CCR2 = (uint32_t) ((htim->Instance->ARR) * dutycycle);
	}
}

void motor_start(char wheel) {
	motor_setPWM(wheel, 0.0);
	if (wheel == 'R') {
		HAL_TIM_PWM_Start(htim, right_channel);
	} else if (wheel == 'L') {
		HAL_TIM_PWM_Start(htim, left_channel);
	}
}

void motor_stop(char wheel) {
	motor_setPWM(wheel, 0.0);
	//Fast motor stop
	if (wheel == 'R') {
		HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, GPIO_PIN_SET);
	} else if (wheel == 'L') {
		HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, GPIO_PIN_SET);
	}

}

void motor_setDirection(char wheel, uint8_t dir) {
	if (wheel == 'R') {
		if (dir == 1) {
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, GPIO_PIN_RESET);
		}
		if (dir == 0) {
			HAL_GPIO_WritePin(DIR_R2_GPIO_Port, DIR_R2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_R1_GPIO_Port, DIR_R1_Pin, GPIO_PIN_RESET);
		}
	} else if (wheel == 'L') {
		if (dir == 1) {
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, GPIO_PIN_RESET);
		}
		if (dir == 0) {
			HAL_GPIO_WritePin(DIR_L2_GPIO_Port, DIR_L2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(DIR_L1_GPIO_Port, DIR_L1_Pin, GPIO_PIN_RESET);
		}
	}
}
