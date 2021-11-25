/*
 * circle_queue.h
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */

#ifndef CIRCLE_QUEUE_STRUCT_H
#define CIRCLE_QUEUE_STRUCT_H


#include <stdint.h>
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "stm32l4xx_hal.h"
#include "MPU6050.h"

#define SIZE_OF_STRUCTQUEUE 160
#define PACKAGE_SIZE 8

struct CAN_QUEUE_DATA {
	uint32_t ID;
	uint8_t data[PACKAGE_SIZE];
};


struct StructQueue {
	uint16_t pointRD, pointWR, queueLength;
	struct CAN_QUEUE_DATA* queue;
};


int StructQueueFull(struct StructQueue *q);
int StructQueueEmpty(struct StructQueue *q);
int EnterStructQueue(struct StructQueue *q, struct CAN_QUEUE_DATA *data);
int LeaveStructQueue(struct StructQueue *q, struct CAN_QUEUE_DATA *data);
int UnreadElements(struct StructQueue *q);

#endif /* CIRCLE_QUEUE_STRUCT */
