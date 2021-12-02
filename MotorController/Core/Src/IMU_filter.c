/*
 * IMU_filter.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Mikkel S. Hansen
 */

#include "IMU_filter.h"

//LP filter coefficients. Calculate based on T and W_c
float A = 1.1403* pow(10, -4);
float B = 1.1891* pow(10, -4);
float C = 0;
float D = 0.8819;
float E = -2.7565;
float F = 2.8744;


// Known filter response sequences
float filt_resp_f250_kf250[20] = { 0, 0, 0.581295066636667, 0.442945217244330,
		-0.548697846654317, -0.514458768708141, 0.531283458233830,
		0.526628135142305, -0.525216276051722, -0.527975362294529,
		0.523703975232896, 0.527968506554599, -0.523390663877524,
		-0.527907349238351, 0.523337192993811, 0.527884452478231,
		-0.523330438599569, -0.527878405984266, 0.523330186303493,
		/*0.527876100986305};*/0.527877100986305 };



// Function prototypes
float LP_filter(float x_old2, float x_old1, float x, float y_old3, float y_old2,
		float y_old1);
int8_t samples_equivalence_test(float *a, float *b, uint32_t len,
		float tolerance);



float LP_filter(float x_old2, float x_old1, float x, float y_old3, float y_old2,
		float y_old1) {
	return A * x_old2 + B * x_old1 + C * x + D * y_old3 + E * y_old2
			+ F * y_old1;
}

int8_t IMU_LP_Filter_test(double signal_freq, double sample_freq,
		float *comp_seq, Vector3Queue *rawQueue, Vector3Queue *filtQueue) {
	double sample_time = 1.0 / sample_freq;

	if (MPU_QUEUE_LENGTH != 20)
		Error_Handler();

	for (uint32_t i = 0; i < MPU_QUEUE_LENGTH; i++) {

		// Generate input sequence
		double val = sin(2.0 * M_PI * signal_freq * i * sample_time);
		Vector3 axisVal = { val, val, val };
		AppendQueue(rawQueue, &axisVal);

		// Compute output sequence
		IMU_LP_Filter_calc_next(rawQueue, filtQueue);
	}

	// Compare output sequences
	float x[MPU_QUEUE_LENGTH] = { 0 };
	float y[MPU_QUEUE_LENGTH] = { 0 };
	float z[MPU_QUEUE_LENGTH] = { 0 };

	for (uint32_t i = 0; i < MPU_QUEUE_LENGTH; i++) {
		x[i] = filtQueue->queue[i].x;
		y[i] = filtQueue->queue[i].y;
		z[i] = filtQueue->queue[i].z;
	}

	return samples_equivalence_test(x, comp_seq, MPU_QUEUE_LENGTH, 0.0000001)
			|| samples_equivalence_test(y, comp_seq, MPU_QUEUE_LENGTH,
					0.0000001)
			|| samples_equivalence_test(z, comp_seq, MPU_QUEUE_LENGTH,
					0.0000001);
}

Vector3 IMU_LP_Filter_calc_next(Vector3Queue *rawQueue, Vector3Queue *filtQueue) {

	//This will be the new filtered sample
	Vector3 next = { 0 };

	//A struct that contain the tree
	Vector3 filtered[3] = { 0 };
	Vector3 raw[3] = { 0 };

	// Iterate through queue starting with the most recent value
	// raw[0] will be the most recent
	for (int8_t i = 0; i < 3; i++) {
		raw[i] = rawQueue->queue[RecentEntryIndex(rawQueue, -i)];
	}

	// Iterate through queue starting with the most recent value
	// filtered[0] will be the most recent
	for (int8_t i = 0; i < 3; i++) {
		filtered[i] = filtQueue->queue[RecentEntryIndex(filtQueue, -i)];
	}

	next.x = LP_filter(raw[2].x, raw[1].x, raw[0].x, filtered[2].x,
			filtered[1].x, filtered[0].x);
	next.y = LP_filter(raw[2].y, raw[1].y, raw[0].y, filtered[2].y,
			filtered[1].y, filtered[0].y);
	next.z = LP_filter(raw[2].z, raw[1].z, raw[0].z, filtered[2].z,
			filtered[1].z, filtered[0].z);

	return next;
}

int8_t samples_equivalence_test(float *a, float *b, uint32_t len,
		float tolerance) {
	for (uint32_t i = 0; i < len; i++) {
		float diff = a[i] - b[i];
		if (diff < 0)
			diff = -diff; // Make sign positive
		if (!(diff <= tolerance))
			return 0;
	}

	return 1;
}

