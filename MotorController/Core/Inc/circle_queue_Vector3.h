/*
 * circle_queue.h
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */

#ifndef CIRCLE_QUEUE_STRUCT_H
#define CIRCLE_QUEUE_STRUCT_H

#include "stdint.h"
#include "Vector3.h"

typedef struct StructQueue {
	uint16_t pointRD, pointWR, queueLength;
	Vector3* queue;
} Vector3Queue;

int Vector3QueueFull(Vector3Queue *q);
int Vector3QueueEmpty(Vector3Queue *q);
int AppendVector3Queue(Vector3Queue *q, Vector3 *data);
int ReadVector3Queue(Vector3Queue *q, Vector3 *data);
int NewestEntryIndex(Vector3Queue *q);
int RecentEntryIndex(Vector3Queue *q, int offset);
int UnreadElements(Vector3Queue *q);

#endif /* CIRCLE_QUEUE_STRUCT */
