/*
 * Copyright (C) Hrubos, Anita
 *
 * This file is part of paparazzi
 *
 */
/**
 * @file "modules/pos_control/pos_control.h"
 * @author Hrubos, Anita
 * Module to control position or velocity
 */

#ifndef POS_CONTROL_H
#define POS_CONTROL_H

// bool to switch on the controller only at the desired time
extern bool pos_control_start;

struct pos_control_s {
	float pitch;
	float roll;
	float yaw;
};

extern struct pos_control_s control_set_point;

extern void pos_control_init(void);
extern void pos_control_periodic(void);

#endif

