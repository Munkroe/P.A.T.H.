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

#define IMU_TX_BUFFER_SIZE 14

int8_t IMU_init();

int8_t IMU_RetrieveData();

int8_t IMU_RequestData();

int8_t IMU_TransmitData();

int8_t IMU_HandleReceivedData();

Vector3 ConvertAccelData(uint8_t *regs);

Vector3 ConvertGyroData(uint8_t *regs);

#endif /* INC_IMU_HANDLER_H_ */
