/**
 * @file "modules/kalman/kalman.h"
 * @author Bodnar, David
 * Kalman-filter for position, velocity, acceleration estimation in x, y, z directions
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdint.h>
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/kalman/libfixmath/fix16.h"

extern bool kalman_take_off;

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
extern void meas_pos_init(void);

extern void send_kalman_telemetry(struct transport_tx *trans, struct link_device* link);

#endif
