/*
 * comm_relay.c
 *
 *  Created on: 28. apr. 2021
 *      Author: Mikkel
 */

#include "comm_relay.h"
#include "stdint.h"
#include "main.h"

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

int from_frame(const char * frame, size_t len, char * destination, uint32_t *outputLen) {
	uint32_t indexFrame = 0, indexDest = 0;

	if (frame[0] == COMM_DEL_START) indexFrame++;

	for (; indexFrame < len - 1; indexFrame++, indexDest++) {
		char c = 0;

		if (frame[indexFrame] == COMM_DEL_START) return -1; // If we meet start delimiter inside frame data, something's wrong.
		if (frame[indexFrame] == COMM_DEL_STOP) return 1; // If we meet stop delimiter inside frame data, it is just a shorter message.

		if (frame[indexFrame] == COMM_ESCAPE) {
			c = frame[indexFrame+1] - 2; // Return the character after the escape character minus 2
			indexFrame++;
		}
		else c = frame[indexFrame]; // There was no escape character, so return it

		destination[indexDest] = c; 	// Insert the data
	}

	// Check whether the last character is either the specified stop delimiter or '0'
	if (indexFrame < len) {
		if (!(frame[indexFrame] == COMM_DEL_STOP || frame[indexFrame] == 0)) return -1;
	}

	*outputLen = indexDest;

	return 1;
}
