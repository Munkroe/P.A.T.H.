/*
 * IMU_filter.h
 *
 *  Created on: Nov 30, 2021
 *      Author: Mikkel S. Hansen
 */

#ifndef INC_IMU_FILTER_H_
#define INC_IMU_FILTER_H_

#include "main.h"
#include "circle_queue_Vector3.h"

int8_t IMU_LP_Filter_test(double signal_freq, double sample_freq,
		float *comp_seq, Vector3Queue *rawQueue, Vector3Queue *filtQueue);

Vector3 IMU_LP_Filter_calc_next(Vector3Queue *rawQueue, Vector3Queue *filtQueue);

#endif /* INC_IMU_FILTER_H_ */
