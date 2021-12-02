/*
 * circle_queue_prototype.h
 *
 *  Created on: Dec 2, 2021
 *      Author: Mikkel S. Hansen
 */

#ifndef INC_CIRCLE_QUEUE_PROTOTYPE_H_
#define INC_CIRCLE_QUEUE_PROTOTYPE_H_

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

#define GENERATE_STRUCT_QUEUE_HEADER(PREFIXQueue, ENTRY_T)							\
																				\
																				\
typedef struct PREFIXQueue {													\
	uint16_t pointRD, pointWR, queueLength;										\
	ENTRY_T* queue;																\
	void (*full_cb_ptr) (PREFIXQueue); /* Queue full callback function pointer*/\
} PREFIXQueue;																	\
																				\
bool IsQueueFull(PREFIXQueue *q);												\
bool IsQueueEmpty(PREFIXQueue *q);												\
int AppendQueue(PREFIXQueue *q, ENTRY_T *data);									\
int ReadFromQueue(PREFIXQueue *q, ENTRY_T *data);								\
int NewestEntryIndex(PREFIXQueue *q);											\
int RecentEntryIndex(PREFIXQueue *q, int offset);								\
int UnreadElements(PREFIXQueue *q);												\

#endif /* INC_CIRCLE_QUEUE_PROTOTYPE_H_ */
