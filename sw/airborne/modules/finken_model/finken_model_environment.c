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

#include "modules/finken_model/finken_model_environment.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"

struct environment_model_s finken_environment_model;

void finken_environment_model_init(void) {
  finken_environment_model.alpha    = 0.0;
  finken_environment_model.distance = 0;
	finken_system_set_point.distance_z     = 0.7;
	finken_system_set_point.velocity_theta = 0.0;
	finken_system_set_point.velocity_x     = 0.0;
	finken_system_set_point.velocity_y     = 0.0;

  register_periodic_telemetry(DefaultPeriodic, "FINKEN_ENVIRONMENT_MODEL", send_finken_environment_model_telemetry);
}

void finken_environment_model_periodic(void){
  int16_t distance = finken_sensor_model.distance_d_front;
  float alpha = 0.0;

  if(finken_sensor_model.distance_d_right < distance)
  {
    distance = finken_sensor_model.distance_d_right;
    alpha = 90.0;
  }

  if(finken_sensor_model.distance_d_back < distance)
  {
    distance = finken_sensor_model.distance_d_back;
    alpha = 180.0;
  }

  if(finken_sensor_model.distance_d_left < distance)
  {
    distance = finken_sensor_model.distance_d_left;
    alpha = 270.0;
  }

  finken_environment_model.distance = distance;
  finken_environment_model.alpha    = alpha;
}

void send_finken_environment_model_telemetry(struct transport_tx *trans, struct link_device* link)
{
  trans=trans;
  link=link;
  DOWNLINK_SEND_FINKEN_ENVIRONMENT_MODEL(
    DefaultChannel,
    DefaultDevice,
    &finken_environment_model.alpha,
    &finken_environment_model.distance
  );
}
