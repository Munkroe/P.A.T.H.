/*
 * circle_queue_prototype.c
 *
 *  Created on: Dec 2, 2021
 *      Author: Mikkel S. Hansen
 */

#include "circle_queue_prototype.h"

#define GENERATE_STRUCT_QUEUE_SOURCE(PREFIXQueue, ENTRY_T)								\
																					\
bool IsQueueFull(PREFIXQueue *q) {													\
	return (((q->pointWR) % q->queueLength) == q->pointRD && (q->pointWR) != 0);	\
}																					\
\
bool IsQueueEmpty(PREFIXQueue *q) {\
	return (q->pointWR == q->pointRD);\
}\
\
int AppendQueue(PREFIXQueue *q, ENTRY_T *data) {\
\
	if (IsQueueFull(q)) {\
		return 0;\
	} else {\
		q->queue[q->pointWR] = *data;\
\
		if ((q->pointWR + 1) == q->queueLength) {\
			q->pointWR = 0;\
			if (q->full_cb_ptr != NULL) q->full_cb_ptr(q); /*Queue full callback*/\
		} else {\
			q->pointWR += 1;\
		}\
\
	}\
	return 1;\
}\
\
int ReadFromQueue(PREFIXQueue *q, ENTRY_T *data) {\
	if (IsQueueEmpty(q)) {\
		return 0;\
	} else {\
		*data = q->queue[q->pointRD];\
		if ((q->pointRD + 1) == q->queueLength) {\
			q->pointRD = 0;\
		} else {\
			q->pointRD += 1;\
		}\
	}\
	return 1;\
}\
\
int NewestEntryIndex(PREFIXQueue *q) {\
	RecentEntryIndex(q, 0);\
}\
\
int RecentEntryIndex(PREFIXQueue *q, int offset) {\
	int i = (q->pointWR) - 1 + offset;				\
	if (i < 0)\
		i = (i % q->queueLength) + q->queueLength;\
	return i;\
}\
\
int UnreadElements(PREFIXQueue *q) {\
	if (q->pointRD == q->pointWR) {\
		return 0;\
	} else {\
		return 1;\
	}\
}\
