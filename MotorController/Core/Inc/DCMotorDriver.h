/*
 * DCMotorDriver.h
 *
 *  Created on: Apr 1, 2021
 *      Author: Mikkel
 */

#ifndef SRC_DCMOTORDRIVER_H_
#define SRC_DCMOTORDRIVER_H_

#include "main.h"

void motor_initi(TIM_HandleTypeDef *htimer, uint32_t r_channel, uint32_t l_channel);

void motor_setPWM(char wheel, float dutycycle);

void motor_start(char wheel);

void motor_stop(char wheel);

void motor_setDirection(char wheel, uint8_t dir);

#endif /* SRC_DCMOTORDRIVER_H_ */
