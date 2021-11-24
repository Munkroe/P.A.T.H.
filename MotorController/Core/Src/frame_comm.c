/*
 * comm_relay.c
 *
 *  Created on: 28. apr. 2021
 *      Author: Mikkel
 */

#include <frame_comm.h>
#include "stdint.h"
#include "main.h"

int is_special_character(char c);

int8_t uart_init_tx(UartCommHandler *handler) {

	if (handler->huart->Init.Mode & UART_MODE_TX_RX
			|| handler->huart->Init.Mode & UART_MODE_TX) {

		// Enable TX
		handler->uart_tx_dmaStart = 0;
		handler->uart_tx_dmaEnd = 0;
		handler->uart_tx_queueEnd = 0;
	} else
		return 0;
	return 1;
}

int8_t uart_init_rx(UartCommHandler *handler) {

	if (handler->huart->Init.Mode & UART_MODE_TX_RX
			|| handler->huart->Init.Mode & UART_MODE_RX) {
		// Enable RX

		if (HAL_UART_Receive_DMA(handler->huart, handler->buffer,
				handler->bufferSize) != HAL_OK)
			return 0;

		handler->validStartDelimiter = false;
		handler->uart_rx_lastStart = -COMM_MAX_FRAME_SIZE;
		handler->uart_rx_read_ptr = 0;
		handler->uart_dma_laps_ahead = 0;
		handler->uart_rx_escapes = 0;
	} else
		return 0;

	return 1;
}

int to_frame(char *frame, char *msg, size_t len) {

//	int i = 2, j = 0, PACKAGE_SIZE = 0;
//
//	frame[0] = COMM_DEL_START;
//	frame[1] = ID;
//
//	if (ID == 3) {
//		PACKAGE_SIZE = 5;
//	} else if (ID == 2) {
//		PACKAGE_SIZE = 24;
//	}
//
//	for (; i < FRAME_SIZE && j < PACKAGE_SIZE; i++, j++) {
//		char c;
//		c = *(revolutionAddr + j);
//
//		if (is_special_character(c)) {
//			frame[i] = COMM_ESCAPE;
//			frame[i + 1] = c + 2;
//			i++;
//		} else {
//			frame[i] = c;
//		}
//	}
//
//	frame[i] = COMM_DEL_STOP;
//
//	return 1;

	int i = 1, j = 0;
	frame[0] = COMM_DEL_START;

	for (; i < COMM_MAX_FRAME_SIZE && j < len; i++, j++) {
		char c;
		c = *(msg + j);

		if (is_special_character(c)) {
			frame[i] = COMM_ESCAPE;
			frame[++i] = c + 2;
		} else {
			frame[i] = c;
		}
	}

	frame[i] = COMM_DEL_STOP;

	return 1;
}

int is_special_character(char c) {
	if ((c == COMM_DEL_START) || (c == COMM_DEL_STOP) || (c == COMM_ESCAPE)
			|| (c == 0))
		return 1;
	return 0;
}

int from_frame(const char *frame, size_t len, char *destination,
		uint32_t *outputLen) {
	uint32_t indexFrame = 0, indexDest = 0;

	if (frame[0] == COMM_DEL_START)
		indexFrame++;

	for (; indexFrame < len - 1; indexFrame++, indexDest++) {
		char c = 0;

		if (frame[indexFrame] == COMM_DEL_START)
			return 0; // If we meet start delimiter inside frame data, something's wrong.
		if (frame[indexFrame] == COMM_DEL_STOP)
			return 0; // If we meet stop delimiter inside frame data, it is just a shorter message.

		if (frame[indexFrame] == COMM_ESCAPE) {
			c = frame[indexFrame + 1] - 2; // Return the character after the escape character minus 2
			indexFrame++;
		} else
			c = frame[indexFrame]; // There was no escape character, so return it

		destination[indexDest] = c; 	// Insert the data
	}

	// Check whether the last character is either the specified stop delimiter or '0'
	if (indexFrame < len) {
		if (!(frame[indexFrame] == COMM_DEL_STOP || frame[indexFrame] == 0))
			return 0;
	}

	*outputLen = indexDest;

	return 1;
}

void uart_transmit(UartCommHandler *handler, char *msg, size_t len,
		uint8_t frameid) {
	char frame[COMM_MAX_FRAME_SIZE] = { 0 };
	char data[COMM_MAX_FRAME_SIZE] = { 0 };

	if (handler->frameid_enabled) {
		data[0] = frameid;
		memcpy(data + 1, msg, len);
		len++;
	} else memcpy(data, msg, len);

	to_frame(frame, data, len);

	size_t frameLength = strlen(frame);
	if (frame[frameLength-1] == 0) frameLength++;


}

void uart_tx_append_queue(handler->huart, frame, frameLength) {
	//

	HAL_UART_Transmit_DMA();
}

void uart_rxhandle(UartCommHandler *handler) {

	if (handler->direction != DIRECTION_RX)
		return;

	// The position at which the DMA writes (can be larger than queue size, if DMA is a lap ahead)
	int dma_ptr =
			(handler->bufferSize - handler->huart->hdmarx->Instance->CNDTR)
					+ handler->bufferSize * handler->uart_dma_laps_ahead;

	// dma_ptr - uart_in_read_ptr is the number of unread/uninterpreted bytes in queue
	for (; dma_ptr - handler->uart_rx_read_ptr > 0;
			handler->uart_in_read_ptr++) {

		// If read pointer crosses "queue border"
		if (handler->uart_rx_read_ptr >= handler->bufferSize) {
			handler->uart_rx_read_ptr = 0;
			handler->uart_rx_lastStart -= handler->bufferSize;
			handler->uart_dma_laps_ahead--;
			dma_ptr = (handler->bufferSize
					- handler->huart->hdmarx->Instance->CNDTR)
					+ handler->bufferSize * handler->uart_dma_laps_ahead;
		}

		// If we find the beginning of a message
		if (handler->buffer[handler->uart_rx_read_ptr] == COMM_DEL_START) {
			handler->uart_rx_lastStart = handler->uart_rx_read_ptr;
			handler->uart_rx_escapes = 0;
		} else if (handler->buffer[handler->uart_rx_read_ptr] == COMM_ESCAPE)
			handler->uart_in_escapes++;

		// If we find the end of a message
		else if (handler->buffer[handler->uart_rx_read_ptr] == COMM_DEL_STOP) {

			int frameLength = handler->uart_rx_read_ptr
					- handler->uart_rx_lastStart + 1;

			if (frameLength <= COMM_MAX_FRAME_SIZE) {
				char frame[COMM_MAX_FRAME_SIZE] = { 0 };

				// If the start and stop delimiter are on opposite sides of the "queue border"
				if (handler->uart_rx_lastStart < 0) {
					memcpy(frame,
							handler->buffer + handler->bufferSize
									+ handler->uart_rx_lastStart,
							-handler->uart_rx_lastStart);
					memcpy(frame - handler->uart_rx_lastStart, handler->buffer,
							handler->uart_rx_read_ptr + 1);
				} else
					memcpy(frame, handler->buffer + handler->uart_rx_lastStart,
							frameLength);

				char data[COMM_MAX_FRAME_SIZE] = { 0 };
				char *pData = data;
				uint32_t dataLength = 0;

				if (from_frame(frame, frameLength, pData, &dataLength)) {

					// Extract id
					uint8_t id = 0;
					if (handler->frameid_enabled) {
						id = pData[0];
						pData++;
						dataLength--;
					}

					// Call handler
					(*handler->rxHandleFunc)(pData, dataLength, id);
				}
			}
		}
	}
}

void uart_dma_lap_increase(UartCommHandler *handler) {
	handler->uart_dma_laps_ahead++;
}
