/*
 * orientation.h
 *
 *  Created on: 4. nov. 2021
 *      Author: Sophus
 */

#ifndef SRC_ORIENTATION_H_
#define SRC_ORIENTATION_H_

#include <frame_comm.h>
#include "stdint.h"
#include "main.h"

extern UART_HandleTypeDef huart2;

#define TOPENCODERRESOLUTION 736 // 192 teeth, each photointerruptor measures both falling and rising (times 2), "2*" because of two photointerruptors measuring with the same resolution
#define UART_ID_ORIENTATION 3

void orientation_reset();
void checkOrientClock();
void calcOrientOutput();
void checkOrientCounterClock();
void checkRevolutionsOrient();
void sendOrientData();
void packOrient();

#endif /* SRC_ORIENTATION_H_ */
