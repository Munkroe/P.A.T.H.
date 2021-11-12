/*
 * comm_relay.c
 *
 *  Created on: 28. apr. 2021
 *      Author: Mikkel
 */

#include "comm_relay.h"
#include "stdint.h"

int to_frame(char * frame, uint8_t *revolutionAddr, uint8_t *ID) {

	int i = 2, j = 0, PACKAGE_SIZE = 0;

	frame[0] = COMM_DEL_START;
	frame[1] = ID;

	if (ID == 3) {
		PACKAGE_SIZE = 5;
	} else if (ID == 2) {
		PACKAGE_SIZE = 24;
	}

	for (; i < FRAME_SIZE && j < PACKAGE_SIZE; i++, j++) {
		char c;
		c = *(revolutionAddr + j);

		if (is_special_character(c)) {
			frame[i] = COMM_ESCAPE;
			frame[i+1] = c + 2;
			i++;
		} else {
			frame[i] = c;
		}
	}

	frame[i] = COMM_DEL_STOP;

	return 1;
}


int is_special_character(char c) {
	if ((c == COMM_DEL_START) || (c == COMM_DEL_STOP) || (c == COMM_ESCAPE) || (c == 0)) return 1;
	return 0;
}

int from_frame(const char * frame, size_t len, struct CAN_QUEUE_DATA * package) {
	int i = 0, j = 0;

	if (frame[0] == COMM_DEL_START) i++;

	for (; i < len - 1; i++, j++) {
		char c = 0;

		if (j >= PACKAGE_SIZE + 1) return -1;

		if (frame[i] == COMM_DEL_START) return -1; // If we meet start delimiter inside frame data, something's wrong.
		if (frame[i] == COMM_DEL_STOP) return 1; // If we meet stop delimiter inside frame data, it is just a shorter message.

		if (frame[i] == COMM_ESCAPE) {
			c = frame[i+1] - 2; // Return the character after the escape character minus 2
			i++;
		}
		else c = frame[i]; // The was no escape character, so return it



		if (j == 0) package->ID = c; // If we are iterating the ID
		else package->data[j-1] = c; 	// If we are iterating the data
	}

	if (i < len) {
		if (!(frame[i] == COMM_DEL_STOP || frame[i] == 0)) return -1;
	}
	return 1;
}
