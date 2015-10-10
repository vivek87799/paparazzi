/*
 * Copyright (C) 2014 Andreas Pfohl
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "modules/finken_model/finken_model_system.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_actuators.h"

#include "subsystems/radio_control.h"
#include "arch/stm32/subsystems/radio_control/spektrum_arch.h"

#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

#include "subsystems/datalink/downlink.h"

/*
 *	P and D gains for the velocity control in x and y direction
 */
#ifndef FINKEN_VELOCITY_X_P
#define FINKEN_VELOCITY_X_P 28
#endif

#ifndef FINKEN_VELOCITY_Y_P
#define FINKEN_VELOCITY_Y_P 28
#endif

#ifndef FINKEN_VELOCITY_X_D
#define FINKEN_VELOCITY_X_D 0
#endif

#ifndef FINKEN_VELOCITY_Y_D
#define FINKEN_VELOCITY_Y_D 0
#endif

/*
 *	P and D gains for the position control in x and y direction
 */

#ifndef FINKEN_POSITION_X_P
#define FINKEN_POSITION_X_P 20
#endif

#ifndef FINKEN_POSITION_Y_P
#define FINKEN_POSITION_Y_P 20
#endif

#ifndef FINKEN_POSITION_X_D
#define FINKEN_POSITION_X_D 0
#endif

#ifndef FINKEN_POSITION_Y_D
#define FINKEN_POSITION_Y_D 0
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
#define FINKEN_POSITION_CONTROL_MODE 0	//2: Circle, 1: position control
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

#define T 1.0f/FINKEN_SYSTEM_UPDATE_FREQ
#define T1 FINKEN_HEIGHT_CONTROL_DELAY_TIME
#define Tn FINKEN_HEIGHT_CONTROL_FOLLOW_TIME
#define Tv FINKEN_HEIGHT_CONTROL_HOLD_TIME
#define Kp FINKEN_HEIGHT_CONTROL_GAIN

#define a0 (2.0f*T1-T)/(2.0f*T1+T)
#define a1 (-4.0f*T1)/(2.0f*T1+T)
#define b0 (T*T-2.0f*Tn*T-2.0f*Tv*T+4.0f*Tn*Tv)/(2.0f*Tn*T+4.0f*T1*Tn)*Kp
#define b1 (T*T-4.0f*Tn*Tv)/(Tn*T+2.0f*T1*Tn)*Kp
#define b2 (T*T+2.0f*Tn*T+2.0f*Tv*T+4.0f*Tn*Tv)/(2.0f*Tn*T+4.0f*T1*Tn)*Kp

struct system_model_s finken_system_set_point;
bool finken_system_model_control_height;

float thrust_k_dec1 = 0.0;
float thrust_k_dec2 = 0.0;
float error_z_k_dec1 = 0.0;
float error_z_k_dec2 = 0.0;

static const float maxRoll   = 20.0f;
static const float maxPitch  = 20.0f;
static const float maxYaw    = 20.0f;
static const float deadRoll  =  1.0f;
static const float deadPitch =  1.0f;
static const float deadYaw   =  1.0f;

float error_vx_p = 0.0;
float error_vy_p = 0.0;
float error_vx_d = 0.0;
float error_vy_d = 0.0;

float error_px_p = 0.0;
float error_py_p = 0.0;
float error_px_d = 0.0;
float error_py_d = 0.0;

float set_point_position_x = 0.0;
float set_point_position_y = 0.0;

uint16_t step = 0;

void finken_system_model_init(void) {
	finken_system_set_point.z          = 0.0;
 	finken_system_set_point.yaw        = 0.0;
	finken_system_model_control_height = 0;
	finken_system_set_point.velocity_x = FINKEN_VELOCITY_DESIRED_Y;
	finken_system_set_point.velocity_y = FINKEN_VELOCITY_DESIRED_X;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_SET_POINT", send_finken_system_set_point_telemetry);

}

/*
 * Use finken_system_set_point to calculate new actuator settings
 */

float takeoff_roll, takeoff_pitch, takeoff_jaw;

void finken_system_model_periodic(void)
{	
	takeoff_roll  = (float) radio_control.values[RADIO_ROLL] / 13000.0 * maxRoll;
	takeoff_pitch = (float) radio_control.values[RADIO_PITCH] / 13000.0 * maxPitch;
	takeoff_jaw   = (float) radio_control.values[RADIO_YAW] / 13000.0 * maxYaw;

/*
 * Allows manual remote control in autopilot mode
 */

	finken_actuators_set_point.roll = takeoff_roll;
	finken_actuators_set_point.pitch = takeoff_pitch;

	if(finken_actuators_set_point.roll < deadRoll && finken_actuators_set_point.roll > -deadRoll)
		finken_actuators_set_point.roll = 0.0f;
	if(finken_actuators_set_point.roll > maxRoll)
		finken_actuators_set_point.roll = maxRoll;
	if(finken_actuators_set_point.pitch < deadPitch&& finken_actuators_set_point.pitch > -deadPitch)
		finken_actuators_set_point.pitch = 0.0f;
	if(finken_actuators_set_point.pitch > maxPitch)
		finken_actuators_set_point.pitch = maxPitch;
	if(finken_actuators_set_point.yaw < deadPitch && finken_actuators_set_point.yaw > -deadYaw)
		finken_actuators_set_point.yaw = 0.0f;
	if(finken_actuators_set_point.yaw > maxYaw)
		finken_actuators_set_point.yaw = maxYaw; 
	
 /*
  *	Altitude controller
  */
 
	float error_z_k = finken_system_set_point.z - POS_FLOAT_OF_BFP(finken_sensor_model.pos.z);

	float thrust_k = -a1 * thrust_k_dec1 - a0 * thrust_k_dec2 + b2 * error_z_k + b1 * error_z_k_dec1 + b0 * error_z_k_dec2;
	
	if( !finken_system_model_control_height )
		thrust_k = 0;

	error_z_k_dec2 = error_z_k_dec1;
	error_z_k_dec1 = error_z_k;
	
	thrust_k_dec2=thrust_k_dec1;
	thrust_k_dec1=thrust_k;

	if (FINKEN_THRUST_DEFAULT + thrust_k /100 < 0.25f || FINKEN_THRUST_DEFAULT + thrust_k / 100 > 1.0f)
		thrust_k -= 2*(b2 + b1 + b0)*(error_z_k+error_z_k_dec1+error_z_k_dec2)/3;

	if(FINKEN_THRUST_DEFAULT + thrust_k / 100 < 0.25f){
		finken_actuators_set_point.thrust = 0.25f;
	}
	else if(FINKEN_THRUST_DEFAULT + thrust_k / 100 > 1.0f){
		finken_actuators_set_point.thrust = 1.0f;
		//TODO anti-windup
	}
	else{
		finken_actuators_set_point.thrust = FINKEN_THRUST_DEFAULT + thrust_k / 100;
	}
 /*
  *	Velocity or position controller
  */

	if(FINKEN_VELOCITY_CONTROL_MODE)	{
		error_vx_p = (finken_system_set_point.velocity_x - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x)) * FINKEN_VELOCITY_X_P;
		error_vy_p = (finken_system_set_point.velocity_y - SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y)) * FINKEN_VELOCITY_Y_P;
		error_vx_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.x)) * FINKEN_VELOCITY_X_D;	//constant velocity
		error_vy_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.y)) * FINKEN_VELOCITY_Y_D;	//constant velocity

		finken_actuators_set_point.pitch = -error_vx_p - error_vx_d;
		if (finken_actuators_set_point.pitch > 20.0f)
			finken_actuators_set_point.pitch = 20.0f;
		else if (finken_actuators_set_point.pitch < -20.0f)
			finken_actuators_set_point.pitch = -20.0f;

		finken_actuators_set_point.roll = error_vy_p + error_vy_d;
		if (finken_actuators_set_point.roll > 20.0f)
			finken_actuators_set_point.roll = 20.0f;
		else if (finken_actuators_set_point.roll < -20.0f)
			finken_actuators_set_point.roll = -20.0f;
 	}
	else if(FINKEN_POSITION_CONTROL_MODE)	{
		switch(FINKEN_POSITION_CONTROL_MODE) {
			case 2:
				// when the quadcopter reaches the start position (x = 1m, y = 0m), start flying in a circle
				// 10% deviation from the set value is accepted
				if(step == 0)	{
					if((finken_sensor_model.pos.x < 0.9 || finken_sensor_model.pos.x > 1.1) && (finken_sensor_model.pos.y < -0.1 || finken_sensor_model.pos.y > 0.1))	{
					set_point_position_x = 1;
					set_point_position_y = 0;
					break;
					}
				}
				step++;
				set_point_position_x = cos(step*6/FINKEN_SYSTEM_UPDATE_FREQ);
				set_point_position_y = sin(step*6/FINKEN_SYSTEM_UPDATE_FREQ);
				break;
			case 1:
				set_point_position_x = FINKEN_POSITION_DESIRED_X;	
				set_point_position_y = FINKEN_POSITION_DESIRED_Y;
				break;
			default:
				break;
		}
		error_px_p = (set_point_position_x - SPEED_FLOAT_OF_BFP(finken_sensor_model.pos.x)) * FINKEN_POSITION_X_P;
		error_py_p = (set_point_position_y - SPEED_FLOAT_OF_BFP(finken_sensor_model.pos.y)) * FINKEN_POSITION_Y_P;
		error_px_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.velocity.x)) * FINKEN_POSITION_X_D;	//zero velocity
		error_py_d = (0 - ACCEL_FLOAT_OF_BFP(finken_sensor_model.velocity.y)) * FINKEN_POSITION_Y_D;	//zero velocity

		if (error_px_p > 20.0f)
			error_px_p = 20.0f;
		else if (error_px_p < -20.0f)
			error_px_p = -20.0f;
		finken_actuators_set_point.pitch = -error_px_p; //+ error_px_d;
		if (error_py_p > 20.0f)
			error_py_p = 20.0f;
		else if (error_py_p < -20.0f)
			error_py_p = -20.0f;
		finken_actuators_set_point.roll = error_py_p;// + error_py_d;
	}		
		else	{
		//finken_actuators_set_point.pitch = 0.0;
		//finken_actuators_set_point.roll  = 0.0;
	}

	// TODO: Theta
}

void send_finken_system_set_point_telemetry(struct transport_tx *trans, struct link_device* link)
{
  trans=trans;
  link=link;
  DOWNLINK_SEND_FINKEN_SYSTEM_SET_POINT(
    DefaultChannel,
    DefaultDevice,
		&finken_system_set_point.z,
		&finken_system_set_point.yaw,
		&set_point_position_x,
		&set_point_position_y
  );
}
