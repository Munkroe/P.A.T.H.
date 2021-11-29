/*
 * circle_queue.h
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */

#ifndef CIRCLE_QUEUE_STRUCT_H
#define CIRCLE_QUEUE_STRUCT_H

#include "MPU6050.h"

//struct CAN_QUEUE_DATA {
//	uint32_t ID;
//	uint8_t data[PACKAGE_SIZE];
//};


typedef struct StructQueue {
	uint16_t pointRD, pointWR, queueLength;
	Axes3* queue;
} StructQueue;


int StructQueueFull(StructQueue *q);
int StructQueueEmpty(StructQueue *q);
int EnterStructQueue(StructQueue *q, Axes3 *data);
int LeaveStructQueue(StructQueue *q, Axes3 *data, uint8_t backtrack);
int UnreadElements(StructQueue *q);

#endif /* CIRCLE_QUEUE_STRUCT */