/*
 * IMU_handler.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Mikkel S. Hansen
 */

#include <circle_queue_Vector3.h>
#include <frame_comm.h>
#include <IMU_handler.h>
#include <main.h>
#include <stm32l4xx_hal_def.h>
#include <stm32l4xx_hal_i2c.h>
#include <string.h>
#include <sys/_stdint.h>
#include <Vector3.h>

extern I2C_HandleTypeDef hi2c3;

IMU_STATE imu_state = IMU_UNINITIALIZED;

uint8_t rx_buffer[IMU_TX_BUFFER_SIZE] = { 0 };

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

uint32_t imu_request_us = 0, imu_request_done_us = 0, imu_retrieve_us = 0, imu_retrieve_done_us = 0, imu_handle_us = 0, imu_handle_done_us = 0;

/**
 * Initiate IMU by configuring registers.
 * @return Succes (1) or fail (0)
 */
int8_t IMU_init() {
	if (imu_state != IMU_UNINITIALIZED) return 0;

	while (MPU_Init(&hi2c3) != HAL_OK) {

	}

	imu_state = IMU_IDLE;
	return 1;
}

int8_t IMU_TransmitComplete() {
	if (imu_state == IMU_REQUEST_START) imu_state = IMU_REQUEST_STOP;
}

int8_t IMU_ReceiveComplete() {
	if (imu_state == IMU_RETRIEVAL_START) imu_state = IMU_RETRIEVAL_STOP;
}

/**
 * Send a command to the IMU setting its I2C read pointer to the sensor data registers
 * @return Success (1) or fail (0)
 */
int8_t IMU_RequestData() {
	if (imu_state == IMU_UNINITIALIZED) return 0;

	imu_request_us = micros();

	if (HAL_I2C_Master_Transmit_IT(&hi2c3, MPU_Address << 1, &MPU_AccelOut, 1)
			!= HAL_OK)
		return 0;

	imu_request_done_us = micros();

	imu_state = IMU_REQUEST_START;

	return 1;
}

/**
 * Send a command to the IMU, ordering it to send back 14 bytes
 * from the I2C read pointer. If IMU_RequestData() has previously been run,
 * register data from accelerometer, temperature and gyroscope is returned.
 * @return Success (1) or fail (0)
 */
int8_t IMU_RetrieveData() {
	if (imu_state != IMU_REQUEST_STOP) return 0;

	imu_retrieve_us = micros();

	memset(rx_buffer, 0, IMU_TX_BUFFER_SIZE); // clear buffer

	if (HAL_I2C_Master_Receive_IT(&hi2c3, MPU_Address << 1, rx_buffer,
	IMU_TX_BUFFER_SIZE))
		return 0;

	imu_retrieve_done_us = micros();

	imu_state = IMU_RETRIEVAL_START;
	return 1;
}

/**
 * Convert the register data retrieved from a previous call to IMU_RetrieveData()
 * into Vector3 format, apply LP filter and add to queues.
 * @return Success (1) or fail (0)
 */
int8_t IMU_HandleReceivedData() {

	if (imu_state != IMU_RETRIEVAL_STOP) return 0;

	imu_handle_us = micros();

	uint8_t accel_regs[6] = { 0 };
	uint8_t temp_regs[2] = { 0 };
	uint8_t gyro_regs[6] = { 0 };

	memcpy(&accel_regs, &rx_buffer, 6);
	memcpy(&temp_regs, &rx_buffer + 6, 2);
	memcpy(&gyro_regs, &rx_buffer + 8, 6);

	// Convert register data to Vector3
	Vector3 newAccel = ConvertAccelData(accel_regs);
	Vector3 newAngVel = ConvertGyroData(gyro_regs);

	// Apply filter and append queues
	AppendVector3Queue(&accelRawQueue, &newAccel);
	//newAccel = IMU_LP_Filter_calc_next(&accelRawQueue, &accelFiltQueue);
	AppendVector3Queue(&accelFiltQueue, &newAccel);

	AppendVector3Queue(&gyroRawQueue, &newAngVel);
	//newAngVel = IMU_LP_Filter_calc_next(&accelRawQueue, &accelFiltQueue);
	AppendVector3Queue(&accelFiltQueue, &newAngVel);

	imu_state = IMU_IDLE;

	imu_handle_done_us = micros();

	return 1;
}

/**
 * Convert gyroscope register data into Vector 3 format
 * @param regs Gyroscope register data (uint8_t[6])
 * @return Gyroscope data in Vector3 format
 */
Vector3 ConvertGyroData(uint8_t *regs) {
	// Data composition from raw data
	int16_t rawGyroData_X = ((int16_t) regs[0] << 8 | regs[1]);
	int16_t rawGyroData_Y = ((int16_t) regs[2] << 8 | regs[3]);
	int16_t rawGyroData_Z = ((int16_t) regs[4] << 8 | regs[5]);

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

	newMPUData.x = (float) rawGyroData_X / sensitivity;
	newMPUData.y = (float) rawGyroData_Y / sensitivity;
	newMPUData.z = (float) rawGyroData_Z / sensitivity;

	return newMPUData;
}

/**
 * Convert accelerometer register data into Vector3 format
 * @param regs Accelerometer register data (uint8_t[6])
 * @return Accelerometer data in Vector3 format
 */
Vector3 ConvertAccelData(uint8_t *regs) {

	// Data composition from raw data
	int16_t rawAccelData_X = ((int16_t) regs[0] << 8 | regs[1]);
	int16_t rawAccelData_Y = ((int16_t) regs[2] << 8 | regs[3]);
	int16_t rawAccelData_Z = ((int16_t) regs[4] << 8 | regs[5]);

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

	newMPUData.x = (float) rawAccelData_X / sensitivity;
	newMPUData.y = (float) rawAccelData_Y / sensitivity;
	newMPUData.z = (float) rawAccelData_Z / sensitivity;

	return newMPUData;
}

/**
 * Send the most recent XY-accelerometer and Z-gyroscope data through UART.
 * @param handler TX UartCommHandler to send IMU data through.
 * @return Succes (1) or fail (0)
 */
int8_t IMU_TransmitData(UartCommHandler *handler) {
	char msg[12] = { 0 };

	Vector3 accel = accelFiltQueue.queue[NewestEntryIndex(&accelFiltQueue)];
	Vector3 gyro = gyroFiltQueue.queue[NewestEntryIndex(&gyroFiltQueue)];

	memcpy(msg, &(accel.x), sizeof(float) * 2); // Accelerometer X and Y
	memcpy(msg + 2 * sizeof(float), &(gyro.z), sizeof(float)); // Gyro Z

	return uart_transmit(handler, msg, sizeof(msg), UART_ID_ACCELGYRO);
}
