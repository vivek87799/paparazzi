/*
 * finken_model_pid.h
 *
 *  Created on: 07.06.2015
 *      Author: Miroslav Slavchev and Jan Sabsch
 */

#ifndef FINKEN_MODEL_PID_H_
#define FINKEN_MODEL_PID_H_

#include <stdint.h>
/*
 * Everything within this class is based on utilities_fun.lua, which should be linked in the wiki.
 */

struct pid_controller {
	float integral;			//i'm not quite sure how this works...
	float previousError;	//the deviation from our desired distance at the last time step. Used to compare with current deviation.
	float min;				//the controller has output boundaries.
	float max;
	float checkMinMax;		//not sure about this one. it doesn't look very important.
	float p, i, d;				//the parameters of the pid. Currently, "i" is always zero, so it should be called a pd_controller instead.
	float res;

	// debug
	float t;
	float pPart, dPart;

	//Ringbuffer mit addition aller i-Anteile
	int k;	//Decides the size of the ringbuffer
	int index; //Shows the position of the next value in the ringbuffer that will be overwritten
	float iPart; //Shows the sum of all components of the ringbuffer
	float ringbuffer[6]; //Saves the values of the last k entries
};

extern void setMinMax(float minParam, float maxParam, struct pid_controller *con);
extern float adjust(float error, float time_step, struct pid_controller *con);
extern void initWallController(struct pid_controller *con);
extern void add_iPart(struct pid_controller *con, float iPart);
extern void initFloatController(struct pid_controller *con);

#endif /* FINKEN_MODEL_PID_H_ */
