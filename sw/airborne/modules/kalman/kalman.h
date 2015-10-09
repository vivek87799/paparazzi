/**
 * @file "modules/kalman/kalman.h"
 * @author Bodnar, David
 * Kalman-filter for position, velocity, acceleration estimation in x, y, z directions
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>

struct state_vector_kalman
{
	int32_t pos_x;
	int32_t pos_y;
	int32_t pos_z;
	int32_t vel_x;
	int32_t vel_y;
	int32_t vel_z;
	int32_t acc_x;
	int32_t acc_y;
	int32_t acc_z;
}; 

extern void update_u(void);
extern void update_z(void);
extern void kalman_init(void);
extern void predict(void);
extern void correct(void);
extern void update_output(void);
extern void kalman_sv_init(void);

#endif
