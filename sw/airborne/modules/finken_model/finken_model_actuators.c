/*
 * Copyright (C) 2014 Sebastian Mai
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

#include "modules/finken_model/finken_model_actuators.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/electrical.h"

struct actuators_model_s finken_actuators_model;
struct actuators_model_s finken_actuators_set_point;


void finken_actuators_model_init(void) {
	finken_actuators_model.pitch  = 0;
	finken_actuators_model.roll   = 0;
	finken_actuators_model.yaw    = 0;
	finken_actuators_model.thrust = 0;

	finken_actuators_set_point.pitch  = 0;
	finken_actuators_set_point.roll   = 0;
	finken_actuators_set_point.yaw    = 0;
	finken_actuators_set_point.thrust = 0;

	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FINKEN_ACTUATORS_MODEL, send_finken_actuators_model_telemetry);
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FINKEN_ACTUATORS_SET_POINT, send_finken_actuators_set_point_telemetry);
}

void finken_actuators_model_periodic(void) {
	finken_actuators_model.pitch  = finken_actuators_set_point.pitch;
	finken_actuators_model.roll   = finken_actuators_set_point.roll;
	finken_actuators_model.yaw    = finken_actuators_set_point.yaw;
	finken_actuators_model.thrust = compensate_battery_drop(finken_actuators_set_point.thrust);
}

/** 
	This function is similar to the compensate_battery_drop function below. It just returns a fix value, if the electrical.vsupply > 121.
*/
float compensate_battery_drop_on_start(float thrust) {
	float defVoltage = 121;
	
	float currVoltage = electrical.vsupply;

	if (currVoltage > 121 )
		return 0.36;
	else if (currVoltage < 105)
		return thrust;
	
	float percentageVloss = 1 - (currVoltage/defVoltage);
	
	float a1 = 37.854887;
	float a2 = -8.288718;
	float a3 = 2.542685;
	float a4 = 0.049967;

	float percentage = (a1* percentageVloss * percentageVloss * percentageVloss) + (a2 * percentageVloss * percentageVloss) + a3 * percentageVloss + a4;

	float comp_thrust = thrust + (thrust * percentage);

	return comp_thrust;
}
/** 
	This function calculates the needed thrust at any given battery voltage.
*/
float compensate_battery_drop(float thrust) {
	float defVoltage = 121;					
	
	float currVoltage = electrical.vsupply;			// cast electrical.vsupply to float to avoid errors by integer division

	if (currVoltage > 121 )					
		return thrust;					// return the current thrust because function is based on values from 12,1V to 10,5V
	else if (currVoltage < 105)
		return thrust;					// return the current thrust because function is based on values from 12,1V to 10,5V
	
	float percentageVloss = 1 - (currVoltage/defVoltage);	// calculation of lost voltage in percent
	
	// Polynomcoefficients calculated by battery drop depandaning on the thrust
	float a1 = 37.854887;
	float a2 = -8.288718;
	float a3 = 2.542685;
	float a4 = 0.049967;
	
	// Polynomial calculation of additionally needed percentage of thrust
	float percentage = (a1* percentageVloss * percentageVloss * percentageVloss) + (a2 * percentageVloss * percentageVloss) + a3 * percentageVloss + a4;

	// adding of the additionally needed thrust	
	float comp_thrust = thrust + (thrust * percentage);

	return comp_thrust;
}
void send_finken_actuators_model_telemetry(struct transport_tx *trans, struct link_device* link)
{
	trans=trans;
	link=link;
	DOWNLINK_SEND_FINKEN_ACTUATORS_MODEL(
		DefaultChannel,
		DefaultDevice,
		&finken_actuators_model.pitch,
		&finken_actuators_model.roll,
		&finken_actuators_model.yaw,
		&finken_actuators_model.thrust
	);
}
void send_finken_actuators_set_point_telemetry(struct transport_tx *trans, struct link_device* link)
{
	trans=trans;
	link=link;
	DOWNLINK_SEND_FINKEN_ACTUATORS_SET_POINT(
		DefaultChannel,
		DefaultDevice,
		&finken_actuators_set_point.pitch,
		&finken_actuators_set_point.roll,
		&finken_actuators_set_point.yaw,
		&finken_actuators_set_point.thrust
	);
}
