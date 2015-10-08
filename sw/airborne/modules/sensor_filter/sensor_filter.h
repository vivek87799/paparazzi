/*
 * Copyright (C) Hrubos, Anita
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/sensor_filter/sensor_filter.h"
 * @author Hrubos, Anita
 * Module to filter and process sensor data.
 */

#ifndef SENSOR_FILTER_H
#define SENSOR_FILTER_H

#include "math/pprz_algebra_int.h"

struct sensor_filter_s {
  struct Int32Vect3 pos;
  struct Int32Quat attitude;
  struct Int32Vect3 velocity;
  struct Int32Rates rates;
  struct Int32Vect3  acceleration;
};

extern struct sensor_filter_s sensor_filtered;

extern void sensor_filter_init(void);
extern void sensor_filter_periodic(void);

#endif

