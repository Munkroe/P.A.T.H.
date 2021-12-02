/*
 * circle-queue.c
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */
#include <circle_queue_Vector3.h>

bool IsQueueFull(Vector3Queue *q) {
	return (((q->pointWR) % q->queueLength) == q->pointRD && (q->pointWR) != 0);
}

bool IsQueueEmpty(Vector3Queue *q) {
	return (q->pointWR == q->pointRD);
}

int AppendQueue(Vector3Queue *q, Vector3 *data) {

	if (IsQueueFull(q)) {
		return 0;
	} else {
		q->queue[q->pointWR] = *data;

		if ((q->pointWR + 1) == q->queueLength) {
			q->pointWR = 0;
			if (q->full_cb_ptr != NULL) q->full_cb_ptr(q); // Queue full callback
		} else {
			q->pointWR += 1;
		}

	}
	return 1;
}

int ReadFromQueue(Vector3Queue *q, Vector3 *data) {
	if (IsQueueEmpty(q)) {
		return 0;
	} else {
		*data = q->queue[q->pointRD];
		if ((q->pointRD + 1) == q->queueLength) {
			q->pointRD = 0;
		} else {
			q->pointRD += 1;
		}
	}
	return 1;
}

int NewestEntryIndex(Vector3Queue *q) {
	RecentEntryIndex(q, 0);
}

int RecentEntryIndex(Vector3Queue *q, int offset) {
	int i = (q->pointWR) - 1 + offset;
	if (i < 0)
		i = (i % q->queueLength) + q->queueLength;
	return i;
}

int UnreadElements(Vector3Queue *q) {
	if (q->pointRD == q->pointWR) {
		return 0;
	} else {
		return 1;
	}
}

//GENERATE_STRUCT_QUEUE_SOURCE(Vector3Queue, Vector3)
