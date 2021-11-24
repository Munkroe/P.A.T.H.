/*
 * comm_relay.h
 *
 *  Created on: 28. apr. 2021
 *      Author: Mikkel
 */

#ifndef SRC_COMM_RELAY_H_
#define SRC_COMM_RELAY_H_

#include "string.h"
#include "stdbool.h"
#include "stdint.h"
#include "stm32l4xx_hal.h"

#define COMM_DEL_START '$'
#define COMM_DEL_STOP '@'
#define COMM_ESCAPE '#'
//#define PACKAGE_SIZE 25
#define FRAME_SIZE 51

#define COMM_MAX_FRAME_SIZE 22

typedef enum COMM_DIRECTION {
	DIRECTION_RX,
	DIRECTION_TX
} COMM_DIRECTION;

typedef struct UartCommHandler {
	COMM_DIRECTION direction;
	char * buffer;
	uint16_t bufferSize;
	UART_HandleTypeDef * huart;

	// Receiver specific
	void (*rxHandlerFunc)(char*, uint32_t); // Pointer "rxHandle" to function with arguments char* and uint32_t
	bool validStartDelimiter;
	int uart_in_lastStart;
	int uart_in_read_ptr;
	int uart_dma_laps_ahead;
	int uart_in_escapes;
} UartCommHandler;

int to_frame(char * frame, uint8_t *revolutionAddr, uint8_t *ID);

int from_frame(const char * frame, size_t len, char * destination, uint32_t *outputLen);

int8_t uart_init_tx(UartCommHandler *handler, UART_HandleTypeDef *huart,
		char *buffer);

int8_t uart_init_rx(UartCommHandler *handler, UART_HandleTypeDef *huart,
		void (*rxHandler)(char*, uint32_t), char *buffer);

void uart_rxhandle(UartCommHandler *handler);

void uart_dma_lap_increase(UartCommHandler * handler);

#endif /* SRC_COMM_RELAY_H_ */
