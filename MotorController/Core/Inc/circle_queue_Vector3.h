/*
 * circle_queue.h
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */

#ifndef CIRCLE_QUEUE_STRUCT_H
#define CIRCLE_QUEUE_STRUCT_H

#include "circle_queue_prototype.h"
#include "stdint.h"
#include "stdbool.h"
#include "Vector3.h"
#include "stddef.h"

typedef struct Vector3Queue {
	uint16_t pointRD, pointWR, queueLength;
	Vector3* queue;
	void (*full_cb_ptr) (Vector3Queue); // Queue full callback function pointer
} Vector3Queue;

bool IsQueueFull(Vector3Queue *q);
bool IsQueueEmpty(Vector3Queue *q);
int AppendQueue(Vector3Queue *q, Vector3 *data);
int ReadFromQueue(Vector3Queue *q, Vector3 *data);
int NewestEntryIndex(Vector3Queue *q);
int RecentEntryIndex(Vector3Queue *q, int offset);
int UnreadElements(Vector3Queue *q);

//GENERATE_STRUCT_QUEUE_HEADER(Vector3Queue, Vector3)

#endif /* CIRCLE_QUEUE_STRUCT */
