/*
 * microsecond_counter.h
 *
 *  Created on: Nov 25, 2021
 *      Author: Mikkel S. Hansen
 */

#ifndef INC_MICROSECOND_COUNTER_H_
#define INC_MICROSECOND_COUNTER_H_

#include "main.h"

#define MICROS_FREQ 1000000

int micros_init(TIM_HandleTypeDef *phtim, uint32_t clk_freq);

uint32_t micros();

uint32_t micros_overflow();


#endif /* INC_MICROSECOND_COUNTER_H_ */



