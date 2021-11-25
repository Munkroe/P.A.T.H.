/*
 * circle-queue.c
 *
 *  Created on: Mar 16, 2021
 *      Author: Mikkel
 */
#include "circle_queue_struct.h"

int StructQueueFull(StructQueue *q){
	return (((q->pointWR +1) % q->queueLength) == q->pointRD);
}

int StructQueueEmpty(StructQueue *q){
	return (q->pointWR == q->pointRD);
}

int EnterStructQueue(StructQueue *q, Axes3 *data) {

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

int LeaveStructQueue(StructQueue *q, Axes3 *data) {
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

int UnreadElements(StructQueue *q){
	if (q->pointRD == q->pointWR){
		return 0;
	}else {
		return 1;
	}
}
