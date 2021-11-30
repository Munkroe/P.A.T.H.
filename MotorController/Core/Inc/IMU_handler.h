/*
 * IMU_handler.h
 *
 *  Created on: Nov 30, 2021
 *      Author: Mikkel S. Hansen
 */

#ifndef INC_IMU_HANDLER_H_
#define INC_IMU_HANDLER_H_

#include "MPU6050.h"
#include "IMU_filter.h"

int8_t IMU_init();

void IMU_RequestData();

int8_t IMU_TransmitData();

int8_t IMU_HandleReceivedData();

#endif /* INC_IMU_HANDLER_H_ */
