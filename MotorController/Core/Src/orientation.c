#include "orientation.h"

uint8_t directionOrient = 0;
int32_t orientIncrement = 0;
float orientAngle = 0.0;
uint8_t spamCheckDirOrient = 0;
float spamCheckOrientAngle = 0.0;
char packedOrientData[50] = { 0 };
uint8_t angularPosition[5] = { 0 };
float angularResolution = 360.0/TOPENCODERRESOLUTION;


void calcOrientOutput() {
	 orientAngle = (orientIncrement % TOPENCODERRESOLUTION) * angularResolution; // Antallet af målte inkrementer ganges med hvor stor en grad hver inkrement er
 }

void packOrient() {
	uint8_t *angularPointer = &directionOrient;
	angularPosition[0] = *(angularPointer);

	angularPointer = &orientAngle;
		for (int x = 1; x < 5 ; x++) {
		angularPosition[x] = *(angularPointer + (x-1));
	}
}

void checkOrientClock() { //A
	if (HAL_GPIO_ReadPin(orientation_clock_GPIO_Port, orientation_clock_Pin)
				== HAL_GPIO_ReadPin(orientation_counterclock_GPIO_Port, orientation_counterclock_Pin)) {
			directionOrient = -1;
			orientIncrement--;//abs((orientIncrement + directionOrient) % TOOTHRESOLUTION);
		} else {

			directionOrient = 1;
			orientIncrement++; //= abs((orientIncrement + directionOrient) % (TOOTHRESOLUTION + 1));
		}

		//checkRevolutionsOrient();
}

void checkOrientCounterClock() { //B
	if (HAL_GPIO_ReadPin(orientation_counterclock_GPIO_Port, orientation_counterclock_Pin)
				== HAL_GPIO_ReadPin(orientation_clock_GPIO_Port, orientation_clock_Pin)) {
			directionOrient = 1;
			orientIncrement++; //= abs((orientIncrement + directionOrient) % TOOTHRESOLUTION);
		} else {
			directionOrient = -1;
			orientIncrement--; //= abs((orientIncrement + directionOrient) % (TOOTHRESOLUTION + 1));
		}

		//checkRevolutionsOrient();
}

//void checkRevolutionsOrient() {
//	if (orientIncrement == 0 && directionOrient == 1) {
//		orientIncrement = 1;
//	}
//	else if (orientIncrement == 0 && directionOrient == -1){
//		orientIncrement = TOOTHRESOLUTION;
//	}
//}

void sendOrientData() {
	if (spamCheckDirOrient != directionOrient || spamCheckOrientAngle != orientAngle) {
		spamCheckDirOrient = directionOrient;
		spamCheckOrientAngle = orientAngle;
		packOrient();
		memset(packedOrientData, 0, sizeof(packedOrientData));

		to_frame(packedOrientData, angularPosition, UART_ID_ORIENTATION);
		HAL_UART_Transmit(&huart2, packedOrientData, sizeof(packedOrientData), HAL_MAX_DELAY);
	}
}
