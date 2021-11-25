/*
 * MPU6050.c
 *
 *  Created on: 18. nov. 2021
 *      Author: micha
 */

#include "MPU6050.h"


float tempVal = 0.0;
Axes3 resultGyro = { 0 };
Axes3 resultAccel = { 0 };

I2C_HandleTypeDef *hi2c;

HAL_StatusTypeDef MPU_Init(I2C_HandleTypeDef *I2C_handler) {
	hi2c = I2C_handler;
	HAL_StatusTypeDef returnValue = HAL_OK;
	uint8_t initializationBuffer[5] = { 0 };

	initializationBuffer[0] = MPU_PWR_MGT_1;
	initializationBuffer[1] = 0x00; //Set clocksource to internal 8MHz oscillator
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1),
			initializationBuffer, 2, 10);
	if (returnValue != HAL_OK)
		return returnValue;

	initializationBuffer[0] = SIGNAL_PATH_RESET;
	initializationBuffer[1] = 0x07; // Resets gyro, accel & temp signal path to disable any filtering
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1),
			initializationBuffer, 2, 10);
	if (returnValue != HAL_OK)
		return returnValue;

	initializationBuffer[0] = DLPF_CFG;
	initializationBuffer[1] = 0x01; //Digital low pass filter disable & gyro sample rate at 1 kHz
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1),
			initializationBuffer, 2, 10);
	if (returnValue != HAL_OK)
		return returnValue;

	initializationBuffer[0] = SMPLRT_DIV;
	initializationBuffer[1] = 0x00; //Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)  @ 1kHz
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1),
			initializationBuffer, 2, 10);
	if (returnValue != HAL_OK)
		return returnValue;

	initializationBuffer[0] = GYRO_CONFIG;
	initializationBuffer[1] = GYRO_CONFIG_SCALE; // Sets the full scale to +-2000 degrees per second
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1),
			initializationBuffer, 2, 10);
	if (returnValue != HAL_OK)
		return returnValue;

	initializationBuffer[0] = ACCEL_CONFIG;
	initializationBuffer[1] = ACCEL_CONFIG_SCALE; // Sets the accelerometer full scale to +-16g
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1),
			initializationBuffer, 2, 10);
	if (returnValue != HAL_OK)
		return returnValue;

	return returnValue;

}

float MPU_Read_Temp() {
	// Enumeration of possible errors
	HAL_StatusTypeDef returnValue;
	int16_t rawTempVal;

	uint8_t tempBuf[2];
	tempBuf[0] = MPU_TempReg;
	returnValue = HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1), tempBuf,
			1, HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
		return returnValue;
	returnValue = HAL_I2C_Master_Receive(hi2c, (MPU_Address << 1) | 0x01,
			tempBuf, 2, HAL_MAX_DELAY);
	if (returnValue != HAL_OK)
		return returnValue;

	// Data composition from raw data
	rawTempVal = ((int16_t) tempBuf[0] << 8 | tempBuf[1]);

	// Temperature calculation
	tempVal = (rawTempVal) / 340.0 + 36.53;

	return tempVal;
}

Axes3 MPU_Read_Gyro() {
	int16_t rawGyroData_X = 0;
	int16_t rawGyroData_Y = 0;
	int16_t rawGyroData_Z = 0;

	float sensitivity = 0.0f;

	if (GYRO_CONFIG_SCALE == 0x00)
		sensitivity = 131.0;
	if (GYRO_CONFIG_SCALE == 0x08)
		sensitivity = 65.5;
	if (GYRO_CONFIG_SCALE == 0x10)
		sensitivity = 32.8;
	if (GYRO_CONFIG_SCALE == 0x18)
		sensitivity = 16.4;

	uint8_t gyroBuf[10];
	gyroBuf[0] = MPU_GyroOut;
	HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1), gyroBuf, 1,
			HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c, (MPU_Address << 1) | 0x01, gyroBuf, 6,
			HAL_MAX_DELAY);

	// Data composition from raw data
	rawGyroData_X = ((int16_t) gyroBuf[0] << 8 | gyroBuf[1]);
	rawGyroData_Y = ((int16_t) gyroBuf[2] << 8 | gyroBuf[3]);
	rawGyroData_Z = ((int16_t) gyroBuf[4] << 8 | gyroBuf[5]);

	resultGyro.x = (float) rawGyroData_X / sensitivity;
	resultGyro.y = (float) rawGyroData_Y / sensitivity;
	resultGyro.z = (float) rawGyroData_Z / sensitivity;

	return resultGyro;
}

Axes3 MPU_Read_Accel() {
	int16_t rawAccelData_X;
	int16_t rawAccelData_Y;
	int16_t rawAccelData_Z;

	float sensitivity;

	if (ACCEL_CONFIG_SCALE == 0x00)
		sensitivity = 16384.0;
	if (ACCEL_CONFIG_SCALE == 0x08)
		sensitivity = 8192.0;
	if (ACCEL_CONFIG_SCALE == 0x10)
		sensitivity = 4096.0;
	if (ACCEL_CONFIG_SCALE == 0x18)
		sensitivity = 2048.0;

	uint8_t accelBuf[10];
	accelBuf[0] = MPU_AccelOut;
	HAL_I2C_Master_Transmit(hi2c, (MPU_Address << 1), accelBuf, 1,
			HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(hi2c, (MPU_Address << 1) | 0x01, accelBuf, 6,
			HAL_MAX_DELAY);

	// Data composition from raw data
	rawAccelData_X = ((int16_t) accelBuf[0] << 8 | accelBuf[1]);
	rawAccelData_Y = ((int16_t) accelBuf[2] << 8 | accelBuf[3]);
	rawAccelData_Z = ((int16_t) accelBuf[4] << 8 | accelBuf[5]);

	resultAccel.x = (float) rawAccelData_X / sensitivity;
	resultAccel.y = (float) rawAccelData_Y / sensitivity;
	resultAccel.z = (float) rawAccelData_Z / sensitivity;

	return resultAccel;
}

void MPU6050() {
	tempVal = MPU_Read_Temp();
	resultAccel = MPU_Read_Accel();
	resultGyro = MPU_Read_Gyro();
}

