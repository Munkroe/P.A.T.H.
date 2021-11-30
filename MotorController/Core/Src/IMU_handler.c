/*
 * IMU_handler.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Mikkel S. Hansen
 */

#include "IMU_handler.h"

extern I2C_HandleTypeDef hi2c3;

uint8_t rx_buffer[1] = { MPU_GyroOut };
uint8_t tx_buffer[6] = { 0 };

Vector3 accelRawArr[MPU_QUEUE_LENGTH] = { 0 };
Vector3 gyroRawArr[MPU_QUEUE_LENGTH] = { 0 };
Vector3Queue accelRawQueue = { .pointRD = 0, .pointWR = 0, .queue = accelRawArr,
		.queueLength = MPU_QUEUE_LENGTH };
Vector3Queue gyroRawQueue = { .pointRD = 0, .pointWR = 0, .queue = gyroRawArr,
		.queueLength = MPU_QUEUE_LENGTH };

Vector3 accelFiltArr[MPU_QUEUE_LENGTH] = { 0 };
Vector3 gyroFiltArr[MPU_QUEUE_LENGTH] = { 0 };
Vector3Queue accelFiltQueue = { .pointRD = 0, .pointWR = 0, .queue =
		accelFiltArr, .queueLength = MPU_QUEUE_LENGTH };
Vector3Queue gyroFiltQueue = { .pointRD = 0, .pointWR = 0, .queue = gyroFiltArr,
		.queueLength = MPU_QUEUE_LENGTH };

int8_t IMU_init() {
	while (MPU_Init(&hi2c3) != HAL_OK) {

	}

	HAL_StatusTypeDef returnValue = HAL_I2C_Master_Transmit_IT(&hi2c3,
			MPU_Address << 1, &MPU_GyroOut, 1);
}

void IMU_RequestData() {
	//	if (globalDMAFlag == 1) {
	//
	//		HAL_I2C_Master_Transmit_IT(&hi2c3, MPU_Address << 1, &MPU_AccelOut, 1);
	//
	//	} else {
	//
	//		HAL_I2C_Master_Transmit_IT(&hi2c3, MPU_Address << 1, &MPU_GyroOut, 1);
	//		globalDMAFlag = 0;
	//		uart_transmit_IMU();
	//	}

	//	if (globalDMAFlag == 0) {
	//		HAL_I2C_Master_Receive_IT(&hi2c3, MPU_Address << 1, MPU_out, 6);
	//		globalDMAFlag = 1;
	//	} else {
	//		HAL_I2C_Master_Receive_IT(&hi2c3, MPU_Address << 1, MPU_out, 6);
	//		globalDMAFlag = 0;
	//	}

}

int8_t IMU_TransmitData(UartCommHandler *handler) {
	char msg[12] = { 0 };

	Vector3 accel = accelFiltQueue.queue[NewestEntryIndex(&accelFiltQueue)];
	Vector3 gyro = gyroFiltQueue.queue[NewestEntryIndex(&gyroFiltQueue)];

	memcpy(msg, &(accel.x), sizeof(float) * 2); // Accelerometer X and Y
	memcpy(msg + 2 * sizeof(float), &(gyro.z), sizeof(float)); // Gyro Z

	return uart_transmit(handler, msg, sizeof(msg), UART_ID_ACCELGYRO);
}

int8_t IMU_HandleReceivedData() {

}

void ProcessGyroData() {
	int16_t rawGyroData_X = 0;
	int16_t rawGyroData_Y = 0;
	int16_t rawGyroData_Z = 0;

	Vector3 newMPUData;

	float sensitivity = 0.0f;

	if (GYRO_CONFIG_SCALE == 0x00)
		sensitivity = 131.0;
	if (GYRO_CONFIG_SCALE == 0x08)
		sensitivity = 65.5;
	if (GYRO_CONFIG_SCALE == 0x10)
		sensitivity = 32.8;
	if (GYRO_CONFIG_SCALE == 0x18)
		sensitivity = 16.4;

	// Data composition from raw data
	rawGyroData_X = ((int16_t) tx_buffer[0] << 8 | tx_buffer[1]);
	rawGyroData_Y = ((int16_t) tx_buffer[2] << 8 | tx_buffer[3]);
	rawGyroData_Z = ((int16_t) tx_buffer[4] << 8 | tx_buffer[5]);

	newMPUData.x = (float) rawGyroData_X / sensitivity;
	newMPUData.y = (float) rawGyroData_Y / sensitivity;
	newMPUData.z = (float) rawGyroData_Z / sensitivity;

	AppendVector3Queue(&gyroRawQueue, &newMPUData);

	Vector3 newFiltVal = newMPUData; // IMU_LP_Filter_calc_next(&gyroRawQueue, &gyroFiltQueue);

	AppendVector3Queue(&gyroFiltQueue, &newFiltVal);
}

void ProcessAccelData() {
	int16_t rawAccelData_X;
	int16_t rawAccelData_Y;
	int16_t rawAccelData_Z;

	Vector3 newMPUData;

	float sensitivity;

	if (ACCEL_CONFIG_SCALE == 0x00)
		sensitivity = 16384.0;
	if (ACCEL_CONFIG_SCALE == 0x08)
		sensitivity = 8192.0;
	if (ACCEL_CONFIG_SCALE == 0x10)
		sensitivity = 4096.0;
	if (ACCEL_CONFIG_SCALE == 0x18)
		sensitivity = 2048.0;

	// Data composition from raw data
	rawAccelData_X = ((int16_t) tx_buffer[0] << 8 | tx_buffer[1]);
	rawAccelData_Y = ((int16_t) tx_buffer[2] << 8 | tx_buffer[3]);
	rawAccelData_Z = ((int16_t) tx_buffer[4] << 8 | tx_buffer[5]);

	newMPUData.x = (float) rawAccelData_X / sensitivity;
	newMPUData.y = (float) rawAccelData_Y / sensitivity;
	newMPUData.z = (float) rawAccelData_Z / sensitivity;

	AppendVector3Queue(&accelRawQueue, &newMPUData);

	Vector3 newFiltVal = newMPUData; //IMU_LP_Filter_calc_next(&accelRawQueue, &accelFiltQueue);

	AppendVector3Queue(&accelFiltQueue, &newFiltVal);
}
