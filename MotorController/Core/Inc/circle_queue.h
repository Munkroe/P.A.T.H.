/*
 * circle_queue.h
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */

#ifndef SRC_CIRCLE_QUEUE_H_
#define SRC_CIRCLE_QUEUE_H_

#include <stdint.h>

#define SIZE_OF_QUEUE 25

struct Queue {
	uint16_t pointRD, pointWR;
	uint8_t queue[SIZE_OF_QUEUE];
};
int QueueFull(struct Queue *q);
int QueueEmpty(struct Queue *q);
int EnterQueue(struct Queue *q, uint8_t data);
int LeaveQueue(struct Queue *q, uint8_t *data);


#endif /* SRC_CIRCLE_QUEUE_H_ */
