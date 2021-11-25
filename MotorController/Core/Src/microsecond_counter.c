/*
 * microsecond_counter.c
 *
 *  Created on: Nov 25, 2021
 *      Author: Mikkel S. Hansen
 */

#include "microsecond_counter.h"

TIM_HandleTypeDef *htim;
uint32_t laps_counter = 0;
uint32_t micros_freq = 0;

int micros_init(TIM_HandleTypeDef *phtim, uint32_t clk_freq) {
	htim = phtim;

	int prescaler = phtim->Instance->PSC + 1;
	uint32_t calc_freq = clk_freq / prescaler;

	if (calc_freq != MICROS_FREQ) {
		micros_freq = calc_freq;
		return 0;
	}

	// Sync up with system tick counter from the start
	laps_counter = HAL_GetTick() * HAL_GetTickFreq() - htim->Instance->CNT;

	return 1;
}

uint32_t micros() {
	return laps_counter + htim->Instance->CNT;
}

uint32_t micros_overflow() {
	laps_counter += htim->Instance->ARR;
}
