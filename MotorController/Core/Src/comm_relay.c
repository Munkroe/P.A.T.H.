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

