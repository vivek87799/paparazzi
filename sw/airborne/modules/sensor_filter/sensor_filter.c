/*
 * Copyright (C) Hrubos, Anita
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/sensor_filter/sensor_filter.c"
 * @author Hrubos, Anita
 * Module to filter and process sensor data.
 */

#include "std.h"
#include "modules/sensor_filter/sensor_filter.h"
#include "subsystems/imu.h"

struct sensor_filter_s sensor_filtered;

int32_t accel_temp_x;
int32_t accel_temp_y;

int32_t accel_send_x;
int32_t accel_send_y;

void sensor_filter_init(void)
{
	memset(&sensor_filtered, 0, sizeof(struct sensor_filter_s)); 
	
	accel_temp_x = 0;
	accel_temp_y = 0;
	accel_send_x = 0;
	accel_send_y = 0;

}

void sensor_filter_periodic(void)
{
	//memcpy(&sensor_filtered.acceleration, &imu.accel, sizeof(struct Int32Vect3));

	//local coordinates !!!
	//TODO: transform into global coordinate system
	sensor_filtered.acceleration.x = imu.accel.x;
	sensor_filtered.acceleration.y = imu.accel.y;

	sensor_filtered.acceleration.z = imu.accel.z;

  //exponentially moving average
  //ins_int.ltp_accel.x = accel_meas_ltp.x*350/600 + accel_x_low*250/600;
  //ins_int.ltp_accel.y = accel_meas_ltp.y*350/600+ accel_y_low*250/600;

	//smooth the data
	accel_send_x = (sensor_filtered.acceleration.x >> 4 ) << 4; 
	accel_send_y = (sensor_filtered.acceleration.y >> 4 ) << 4; 

	accel_temp_x = sensor_filtered.acceleration.x;
	accel_temp_y = sensor_filtered.acceleration.y;
}
