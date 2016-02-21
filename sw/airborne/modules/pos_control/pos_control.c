/*
 * Copyright (C) Hrubos, Anita
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/pos_control/pos_control.c"
 * @author Hrubos, Anita
 * Module to control position or velocity
 */

#include "std.h"
#include "modules/pos_control/pos_control.h"
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_actuators.h"

/*
 *	P and D gains for the velocity control in x and y direction
 */
#ifndef FINKEN_VELOCITY_X_P
#define FINKEN_VELOCITY_X_P 0.8
#endif

#ifndef FINKEN_VELOCITY_Y_P
#define FINKEN_VELOCITY_Y_P 0.8
#endif

#ifndef FINKEN_VELOCITY_X_D
#define FINKEN_VELOCITY_X_D 0.01
#endif

#ifndef FINKEN_VELOCITY_Y_D
#define FINKEN_VELOCITY_Y_D 0.01
#endif

/*
 *	P and D gains for the position control in x and y direction
 */

#ifndef FINKEN_POSITION_X_P
#define FINKEN_POSITION_X_P 15
#endif

#ifndef FINKEN_POSITION_Y_P
#define FINKEN_POSITION_Y_P 15
#endif

#ifndef FINKEN_POSITION_X_D
#define FINKEN_POSITION_X_D 1.6
#endif

#ifndef FINKEN_POSITION_Y_D
#define FINKEN_POSITION_Y_D 1.6
#endif

/*
 *	constant to enable/disable velocity controller
 */
#ifndef FINKEN_VELOCITY_CONTROL_MODE
#define FINKEN_VELOCITY_CONTROL_MODE 0
#endif

/*
 *	desired velocity in x and y direction (m/sec)
 */
#ifndef FINKEN_VELOCITY_DESIRED_X
#define FINKEN_VELOCITY_DESIRED_X 0.0	// m/sec
#endif

#ifndef FINKEN_VELOCITY_DESIRED_Y
#define FINKEN_VELOCITY_DESIRED_Y 0.0	// m/sec
#endif

/*
 *	constant to enable/disable position controller
 */
#ifndef FINKEN_POSITION_CONTROL_MODE
#define FINKEN_POSITION_CONTROL_MODE 1
#endif

/*
 *	desired position in x and y direction (m)
 */
#ifndef FINKEN_POSITION_DESIRED_X
#define FINKEN_POSITION_DESIRED_X 0.0	// m
#endif

#ifndef FINKEN_POSITION_DESIRED_Y
#define FINKEN_POSITION_DESIRED_Y 0.0	// m
#endif

static const float maxRoll       = 20.0f;
static const float maxPitch      = 20.0f;
static const float maxYaw        = 20.0f;

float error_vx_p;
float error_vy_p;
float error_vx_d;
float error_vy_d;

float error_px_p;
float error_py_p;
float error_px_d;
float error_py_d;

float velocity_change_x = 0.0;
float velocity_change_y = 0.0;
float last_velocity_x = 0.0;
float last_velocity_y = 0.0;

struct pos_control_s control_set_point;

// bool to switch on the controller only at the desired time
bool pos_control_start;

void pos_control_init(void)
{
	control_set_point.pitch = 0.0;
	control_set_point.roll = 0.0;
	control_set_point.yaw = 0.0;

	pos_control_start = 0;
}

void pos_control_periodic(void)
{
	/*
	*	Velocity or position controller
	*/

	if(pos_control_start)
	{
		if(FINKEN_VELOCITY_CONTROL_MODE)	{

			// difference between the current velocity and the last velocity measured
			velocity_change_x = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x) - last_velocity_x;
			velocity_change_y = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y) - last_velocity_y;

			// errors multiplied by the gains
			error_vx_p = (FINKEN_VELOCITY_DESIRED_X - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x)) * FINKEN_VELOCITY_X_P;
			error_vy_p = (FINKEN_VELOCITY_DESIRED_Y - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y)) * FINKEN_VELOCITY_Y_P;
			// use the acceleration from the IMU			
			error_vx_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.x)) * FINKEN_VELOCITY_X_D;	//constant velocity
			error_vy_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.y)) * FINKEN_VELOCITY_Y_D;	//constant velocity
			// use the velocity difference based on the optical flow measurement
			/*error_vx_d = (0 - velocity_change_x) * FINKEN_VELOCITY_X_D;	//constant velocity
			error_vy_d = (0 - velocity_change_y) * FINKEN_VELOCITY_Y_D;	//constant velocity*/

			// limit the control signal
			control_set_point.pitch += -error_vx_p - error_vx_d;
			if (control_set_point.pitch > maxPitch)
				control_set_point.pitch = maxPitch;
			else if (control_set_point.pitch < -maxPitch)
				control_set_point.pitch = -maxPitch;
			control_set_point.roll += error_vy_p + error_vy_d;
			if (control_set_point.roll > maxRoll)
				control_set_point.roll = maxRoll;
			else if (control_set_point.roll < -maxRoll)
				control_set_point.roll = -maxRoll;

			finken_actuators_set_point.pitch = control_set_point.pitch;
			finken_actuators_set_point.roll = control_set_point.roll;


			last_velocity_x = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x);
			last_velocity_y = SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x);

		} else if(FINKEN_POSITION_CONTROL_MODE)	{

			// errors multiplied by the gains
	 		error_px_p = (FINKEN_POSITION_DESIRED_X - POS_FLOAT_OF_BFP(finken_sensor_model.pos.x)) * FINKEN_POSITION_X_P;
	 		error_py_p = (FINKEN_POSITION_DESIRED_Y - POS_FLOAT_OF_BFP(finken_sensor_model.pos.y)) * FINKEN_POSITION_Y_P;//FINKEN_POSITION_Y_P;
	 		error_px_d = (0 - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x)) * FINKEN_POSITION_X_D;	//zero velocity
	 		error_py_d = (0 - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y)) * FINKEN_POSITION_Y_D;	//zero velocity
	 
			// limit the control signals
	 		control_set_point.pitch = -error_px_p - error_px_d;
	 		if (control_set_point.pitch > maxPitch)
	 			control_set_point.pitch = maxPitch;
	 		else if (control_set_point.pitch < -maxPitch)
	 			control_set_point.pitch = -maxPitch;
	 		control_set_point.roll = error_py_p + error_py_d;
	 		if (control_set_point.roll > maxRoll)
	 			control_set_point.roll = maxRoll;
	 		else if (control_set_point.roll < -maxRoll)
	 			control_set_point.roll = -maxRoll;

			finken_actuators_set_point.pitch = control_set_point.pitch;
			finken_actuators_set_point.roll = control_set_point.roll;
	  }
	}
}
