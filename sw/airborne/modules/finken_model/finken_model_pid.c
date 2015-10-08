/*
 * finken_model_pid.c
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#include "finken_model_pid.h"

/**
 * Set the min an max values of a pid_controller, which is passed as a parameter.
 */

void setMinMax(float minParam, float maxParam, struct pid_controller *con) {
	con->min = minParam;
	con->max = maxParam;
	con->checkMinMax = 1;
}

/*
 * This is the main method of the pid_controller. the error should be the distance to the desired distance.
 */
float adjust(float error, float time_step, struct pid_controller *con) {
	con->t = time_step;
	float derivative = (error - con->previousError) / time_step;

	if (con->previousError == 0) {
		derivative = 0;
	}
	add_iPart(con, error * time_step);
	con->previousError = error;
	con->pPart = con->p * error;
	con->dPart = con->d * derivative;
	float res = con->pPart + con->iPart + con->dPart;

	if (con->checkMinMax == 1) {
		if (res < con->min) {
			res = con->min;
		} else if (res > con->max) {
			res = con->max;
		}
	}
	//res = -res;
	con->res = res;
	return res;
}

void initWallController(struct pid_controller *con) {
	con->p = 2.5;
	con->i = 0;
	con->d = 0.2;
	float cap = 250;
	con->min = -cap;
	con->max = cap;
	con->checkMinMax = 1;
	con->index = 0;
	con->k = 6;
	con->iPart = 0;
}

void initFloatController(struct pid_controller *con) {
	initWallController(con);
	con->p = 0.1;
	con->i = 0.1;
	con->d = 0.1;
	con->checkMinMax = 1;
	float cap = 0;
	con->min = -cap;
	con->max = cap;
}

/*
 * This method allows to add a new i_error into the ringbuffer
 */
extern void add_iPart(struct pid_controller *con, float i_error) {
	con->index = con->index % con->k;
//	con->iPart = con->iPart - con->ringbuffer[con->index] + i_error;i
	con->ringbuffer[con->index] = i_error;
	float sum = 0;
	for (int i = 0; i < con->k; i++){
		sum += con->ringbuffer[i];
	}
	con->iPart = sum * con->i;
	con->index++;
}

void reset(struct pid_controller *con) {
	for (int i = 0; i < con->k; i++) {
		con->ringbuffer[i] = 0;
	}
}

