/*
 * comm_relay.h
 *
 *  Created on: 28. apr. 2021
 *      Author: Mikkel
 */

#ifndef SRC_COMM_RELAY_H_
#define SRC_COMM_RELAY_H_

#include "string.h"
#include "stdint.h"

#define COMM_DEL_START '$'
#define COMM_DEL_STOP '@'
#define COMM_ESCAPE '#'
//#define PACKAGE_SIZE 25
#define FRAME_SIZE 51

#define COMM_MAX_FRAME_SIZE 22

int to_frame(char * frame, uint8_t *revolutionAddr, uint8_t *ID);

int is_special_character(char c);

int from_frame(const char * frame, size_t len, char * destination, uint32_t *outputLen);

#endif /* SRC_COMM_RELAY_H_ */
