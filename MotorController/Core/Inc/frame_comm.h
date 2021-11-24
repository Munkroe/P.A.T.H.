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
	bool frameid_enabled;

	// Receiver specific
	void (*rxHandleFunc)(char*, uint32_t, uint8_t); // Pointer "rxHandle" to function with arguments char* and uint32_t
	int uart_rx_startDel;
	int uart_rx_read_ptr;
	int uart_dma_laps_ahead;
	int uart_rx_escapeDel;

	// Transmitter specific
	uint32_t uart_tx_dmaStart;
	uint32_t uart_tx_dmaEnd;
	uint32_t uart_tx_queueEnd;
} UartCommHandler;

int8_t uart_init_tx(UartCommHandler *handler);

int8_t uart_init_rx(UartCommHandler *handler);

void uart_transmit(UartCommHandler *handler, char *msg, size_t len,
		uint8_t frameid);

void uart_rxhandle(UartCommHandler *handler);

void uart_dma_lap_increase(UartCommHandler * handler);

#endif /* SRC_COMM_RELAY_H_ */
