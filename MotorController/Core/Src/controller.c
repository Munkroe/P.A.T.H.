#include "controller.h"

float T = 0.01;

float calculateError(float ref, float meas) {
	return ref - meas;
}

float nextVoltage(float lastErr, float lastVoltage) {
	return lastErr*2.82*T + lastVoltage;
}


