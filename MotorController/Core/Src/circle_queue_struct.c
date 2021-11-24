/*
 * circle-queue.c
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */
#include "circle_queue_struct.h"

int StructQueueFull(struct StructQueue *q){
	return (((q->pointWR +1) % q->queueLength) == q->pointRD);
}

int StructQueueEmpty(struct StructQueue *q){
	return (q->pointWR == q->pointRD);
}

int EnterStructQueue(struct StructQueue *q, struct CAN_QUEUE_DATA *data) {

	if (StructQueueFull(q)) {
		return 0;
	}
	else {
		q->queue[q->pointWR] = *data;


		if ((q->pointWR + 1) == q->queueLength){
			q->pointWR = 0;
		}
		else{
			q->pointWR += 1;
		}

	}
	return 1;
}

int LeaveStructQueue(struct StructQueue *q, struct CAN_QUEUE_DATA *data) {
	if (StructQueueEmpty(q)){
		return 0;
	}
	else {
		*data = q->queue[q->pointRD];
		if((q->pointRD + 1) == q->queueLength){
			q->pointRD = 0;
		}
		else{
			q->pointRD +=1;
		}
	}
	return 1;
}

int UnreadElements(struct StructQueue *q){
	if (q->pointRD == q->pointWR){
		return 0;
	}else {
		return 1;
	}
}
