#include "float_controller.h"

struct pid_controller xFinkenFloatController;
struct pid_controller yFinkenFloatController;

float oldL, oldR, oldF, oldB;
float timeStep = 0.03;
float cap = 15;

void updateDistances() {
	oldL = finken_sensor_model.distance_d_left;
	oldR = finken_sensor_model.distance_d_right;
	oldB = finken_sensor_model.distance_d_back;
	oldF = finken_sensor_model.distance_d_front;
}

void float_controller_init(void) {
	updateDistances();
	initFloatController(&xFinkenFloatController);
	initFloatController(&yFinkenFloatController);
}

void float_controller_periodic(void) {
	int xVelocity = getXDistanceDiff(); // / timeStep;
	float xAcceleration = adjust(xVelocity, 1, &xFinkenFloatController);
	alphaComponents[2] = xAcceleration;

	int yVelocity = getYDistanceDiff(); // / timeStep;
	float yAcceleration = adjust(yVelocity, 1, &yFinkenFloatController);
	betaComponents[2] = -yAcceleration;

	updateActuators();
	updateDistances();
}

int getXDistanceDiff() {
	if (sonar_values.front < sonar_values.back) {
		return sonar_values.front - oldF;
	} else {
		return -(sonar_values.back - oldB);
	}
}

int getYDistanceDiff() {
	if (sonar_values.left < sonar_values.right) {
		return sonar_values.left - oldL;
	} else {
		return -(sonar_values.right - oldR);
	}
}


