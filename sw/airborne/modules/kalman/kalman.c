/**
 * @file "modules/kalman/kalman.c"
 * @author Bodnar, David
 * Kalman-filter for position, velocity, acceleration estimation in x, y, z directions
 */

#include <stdint.h>
#include <assert.h>
#include "modules/kalman/kalman.h"
#include "modules/kalman/libfixkalman/fixkalman.h"			// fixkalman
#include "modules/kalman/libfixmath/fix16.h"				// conversions to fix16
#include "math/pprz_algebra_int.h"							// fixpoint arithmetic
#include "modules/finken_model/finken_model_actuators.h"
#include "modules/finken_model/finken_model_sensors.h"
#include "subsystems/radio_control.h"						// controll values from controller

// last measurement time to compute time difference
uint32_t last_time;

// last measured vertical positions
struct last_measurement_vertical_positions last_pos;

// struct to communicate results to other modules
struct state_vector_kalman kalman_sv_pva;

#define PI  3.14159265359

// activate LUT for trigonometric functions
#define FIXMATH_SIN_LUT

// disable uncontrolled functions for the kalman filter library
#define KALMAN_DISABLE_UC
#define KALMAN_DISABLE_LAMBDA

// create the filter structure
#define K_NAME K_PVA
#define K_NUM_S 9
#define K_NUM_I 3

// structs of kalman filter (controlled)
kalman16_t k_pva;
kalman16_observation_t k_pva_m;

// create the measurement structure
#define K_MEAS_NAME position
#define K_NUM_MEAS 9

#define matrix_set(matrix, row, column, value) \
    matrix->data[row][column] = value

// global variable for mass
fix16_t m;

// Error if fixmatrix library is not configured correctly (fixmatrix.h FIXMATRIX_MAX_SIZE)
#ifndef FIXMATRIX_MAX_SIZE
#error FIXMATRIX_MAX_SIZE must be defined and greater or equal to the number of states, inputs and measurements.
#endif

#if (FIXMATRIX_MAX_SIZE < K_NUM_S) || (FIXMATRIX_MAX_SIZE < K_NUM_I) || (FIXMATRIX_MAX_SIZE < K_NUM_MEAS)
#error FIXMATRIX_MAX_SIZE must be greater or equal to the number of states, inputs and measurements.
#endif

// init state vector
void kalman_sv_init(void) {
	// init state vector
	kalman_sv_pva.pos_x = 0;
	kalman_sv_pva.pos_y = 0;
	kalman_sv_pva.pos_z = 0;
	kalman_sv_pva.vel_x = 0;
	kalman_sv_pva.vel_y = 0;
	kalman_sv_pva.vel_z = 0;
	kalman_sv_pva.acc_x = 0;
	kalman_sv_pva.acc_y = 0;
	kalman_sv_pva.acc_z = 0;

	// telemetry
	register_periodic_telemetry(DefaultPeriodic, "KALMAN", send_kalman_telemetry);
}

// init vertical positions
void meas_pos_init(void) {
	last_pos.x = 0;
	last_pos.y = 0;
}

// update input vector
void update_u(void) {
	mf16 *u = kalman_get_input_vector(&k_pva);

	// get controll data from finken model
	// Roll --> alpha
	// Pitch --> beta
	// Yaw --> theta

	/*
	// controll data from autopilot
	float alpha = finken_actuators_model.alpha;
	float beta = finken_actuators_model.beta;
	// float theta = finken_actuators_model.theta; // grad/s winkelgeschwindigkeit
	float theta = 0.0;
	float throttle = finken_actuators_model.thrust;
	*/

	// controll data from controller
	float alpha = ((float) radio_control.values[RADIO_ROLL]) / (12236*180) * 45 * PI;
	float beta = ((float) radio_control.values[RADIO_PITCH]) / (12236*180) * 45 * PI;
	//float theta = ((float) radio_control.values[RADIO_YAW]) / (12236*180) * 90 * PI;
	float theta = 0.0;

	// correction to 0 with 1318
	float throttle = (((float)radio_control.values[RADIO_THROTTLE]) + 1318.0) / 12236.0 * 100;

	// handle controller spezified dead zones
	if (alpha<1.0 && alpha>-1.0) {
		alpha = 0.0;
	}
	if (beta<1.0 && beta>-1.0) {
		beta = 0.0;
	}

	// trigonometric variables to reduce future computations
	fix16_t alpha_sin = fix16_sin(fix16_from_float(alpha));
	fix16_t alpha_cos = fix16_cos(fix16_from_float(alpha));

	fix16_t beta_sin = fix16_sin(fix16_from_float(beta));
	fix16_t beta_cos = fix16_cos(fix16_from_float(beta));

	fix16_t theta_sin = fix16_sin(fix16_from_float(theta));
	fix16_t theta_cos = fix16_cos(fix16_from_float(theta));

	fix16_t thrust = fix16_from_float(throttle);

	// Conversionsfunction from Christoph: thrust[g] = 0.01514x^2 + 0.65268x [% --> gramm] corresponding to 4 motors
	fix16_t thrust_converted = fix16_mul(fix16_add(fix16_mul(fix16_from_float(0.01514), fix16_mul(thrust, thrust)),
		fix16_mul(fix16_from_float(0.65268), thrust)), fix16_from_float(4.0));

	// convert to force
	thrust_converted = fix16_mul(thrust_converted, fix16_from_float(9.81));

	// decompore thrust vector to global acceleration vectors and update input vector
	u->data[0][0] = fix16_mul(fix16_add(fix16_mul(theta_cos, fix16_mul(beta_sin, alpha_cos)), 
		fix16_mul(theta_sin, alpha_sin)), thrust_converted);	// Thrust in X-Direction
	u->data[1][0] = fix16_mul(fix16_sub(fix16_mul(theta_sin, fix16_mul(beta_sin, alpha_cos)), 
		fix16_mul(theta_cos, alpha_sin)), thrust_converted);	// Thrust in Y-Direction
	u->data[2][0] = fix16_sub(fix16_mul(fix16_mul(beta_cos, alpha_cos), thrust_converted), 
		fix16_mul(m, fix16_from_float(9.81)));	// Thrust in Z-Direction
}

// update observation vector
void update_z(void) {
	mf16 *z = kalman_get_observation_vector(&k_pva_m);

	// const to reduce computation
	fix16_t helper_const;

	// Time passed since last observation
	uint32_t now = get_sys_time_msec();
	float diff_float = (float) (now - last_time);
	fix16_t diff = fix16_div(fix16_from_float(diff_float), fix16_from_float(1000.0));

	// Euler-Angles
	fix16_t alpha = finken_sensor_attitude.phi * (1<<(16-INT32_ANGLE_FRAC));
	fix16_t beta = finken_sensor_attitude.theta * (1<<(16-INT32_ANGLE_FRAC));
	fix16_t theta = finken_sensor_attitude.psi * (1<<(16-INT32_ANGLE_FRAC));

	// Trigonometric variables to reduce future computation
	fix16_t alpha_sin = fix16_sin(alpha);
	fix16_t alpha_cos = fix16_cos(alpha);

	fix16_t beta_sin = fix16_sin(beta);
	fix16_t beta_cos = fix16_cos(beta);

	fix16_t theta_sin = fix16_sin(theta);
	fix16_t theta_cos = fix16_cos(theta);

	// Rotationsmatrix
	fix16_t R11 = fix16_add(theta_cos, beta_cos);
	fix16_t R12 = fix16_sub(fix16_mul(fix16_mul(theta_cos, beta_sin), alpha_sin), 
		fix16_mul(theta_sin, alpha_cos));
	fix16_t R13 = fix16_add(fix16_mul(fix16_mul(theta_cos, beta_sin), alpha_cos), 
		fix16_mul(theta_sin, alpha_sin));

	fix16_t R21 = fix16_add(theta_sin, beta_cos);
	fix16_t R22 = fix16_add(fix16_mul(fix16_mul(theta_sin, beta_sin), alpha_sin), 
		fix16_mul(theta_cos, alpha_cos));
	fix16_t R23 = fix16_sub(fix16_mul(fix16_mul(theta_sin, beta_sin), alpha_cos), 
		fix16_mul(theta_cos, alpha_sin));

	fix16_t R31 = fix16_mul(beta_sin, fix16_from_float(-1.0));
	fix16_t R32 = fix16_mul(beta_cos, alpha_sin);
	fix16_t R33 = fix16_mul(beta_cos, alpha_cos);

	// get data of flow sensor
	fix16_t velocity_x = finken_sensor_model.velocity.x / (1<<(INT32_SPEED_FRAC-16));
	fix16_t velocity_y = finken_sensor_model.velocity.y / (1<<(INT32_SPEED_FRAC-16));
	fix16_t position_z = finken_sensor_model.pos.z * (1<<(16-INT32_POS_FRAC));

	// scaling because of flow sensor (16/4.6)
	//velocity_x = fix16_mul(velocity_x, fix16_from_float(3.47826087));
	//velocity_y = fix16_mul(velocity_y, fix16_from_float(3.47826087));

	// compute current velocity in x and y direction and hight
	position_z = fix16_mul(R33, position_z);
	helper_const = velocity_x;
	velocity_x = fix16_add(fix16_mul(R11, velocity_x), fix16_mul(R12, velocity_y));
	velocity_y = fix16_add(fix16_mul(R21, helper_const), fix16_mul(R22, velocity_y));

	// indirect computation of position difference to last time in x and y 
	// and velocity in z direction
	fix16_t velocity_z = fix16_div(fix16_sub(position_z, z->data[2][0]), diff);
	fix16_t position_x = fix16_mul(fix16_div(fix16_add(velocity_x, z->data[3][0]), 
		fix16_from_float(2.0)), diff);
	fix16_t position_y = fix16_mul(fix16_div(fix16_add(velocity_y, z->data[4][0]), 
		fix16_from_float(2.0)), diff);

	// get accelerometer data
	fix16_t acceleration_x = finken_sensor_model.acceleration.x * (1<<(16-INT32_ACCEL_FRAC));
	fix16_t acceleration_y = finken_sensor_model.acceleration.y * (1<<(16-INT32_ACCEL_FRAC));
	fix16_t acceleration_z = finken_sensor_model.acceleration.z * (1<<(16-INT32_ACCEL_FRAC));

 	last_pos.x += position_x;
 	last_pos.y += position_y;

	// update measurement vector vector
	z->data[0][0] = last_pos.x;	  // pos_x
  z->data[1][0] = last_pos.y ;	// pos_y
	z->data[2][0] = position_z;		// pos_z
	z->data[3][0] = velocity_x;		// vel_x
  z->data[4][0] = velocity_y;		// vel_y
	z->data[5][0] = velocity_z;		// vel_z
	z->data[6][0] = fix16_add(fix16_add(fix16_mul(R11, acceleration_x),
		fix16_mul(R12, acceleration_y)), fix16_mul(R13, acceleration_z));	// acc_x
  z->data[7][0] = fix16_add(fix16_add(fix16_mul(R21, acceleration_x), 
		fix16_mul(R22, acceleration_y)), fix16_mul(R23, acceleration_z));	// acc_y
  z->data[8][0] = fix16_add(fix16_add(fix16_mul(R31, acceleration_x), 
		fix16_mul(R32, acceleration_y)), fix16_mul(R33, acceleration_z));	// acc_z

	// update timestamp
	last_time = now;
}

// initialize kalman filter (initializer function for paparazzi)
void kalman_init(void) {
	// time and other constants
	const fix16_t dt = fix16_div(fix16_one,fix16_from_float(15.0));		// SET TIME CONSTANT!!!
	const fix16_t dt_2 = fix16_sq(dt);

	// initialisation constants
	m = fix16_from_float(304.0);				// SET MASS!!! [g]
	const fix16_t init_uncert_1 = fix16_from_float(1.0);	// SET INITIAL UNCERTEANTY!!!
	const fix16_t init_uncert_2 = fix16_from_float(0.3);	// SET INITIAL UNCERTEANTY!!!
	const fix16_t init_uncert_3 = fix16_from_float(0.5);	// SET INITIAL UNCERTEANTY!!!
	const fix16_t sigma = fix16_from_float(2.0);			// SET SIGMA!!!
	fix16_t helper_const;									// varible to speed up matrix assignment
	//const fix16_t c = fix16_from_float(0.95);				// c = 0.95 --> ~ as it would be a plane surface
	//const fix16_t A1 = fix16_from_float(0.03947);			// A_x,y = 27.5*27.5 and 12*27,5 rotated with 5 degrees ~ 0.03947
	//const fix16_t A2 = fix16_from_float(0.075625);			// A_z = 27.5*27.5 cm --> maximal distance between endpoints of the rotors

	// init vertical positions
	meas_pos_init();
	
	// init output struct
	kalman_sv_init();

	// init filter
	kalman_filter_initialize(&k_pva, K_NUM_S, K_NUM_I);

	// init observation
	kalman_observation_initialize(&k_pva_m, K_NUM_S, K_NUM_MEAS);

	helper_const = fix16_div(dt_2, fix16_from_float(2.0));

	// get system state transition model matrix from struct
	mf16 *A = kalman_get_state_transition(&k_pva);

	matrix_set(A, 0, 0, fix16_one);
	matrix_set(A, 0, 3, dt);
	matrix_set(A, 0, 6, helper_const);

	matrix_set(A, 1, 1, fix16_one);
	matrix_set(A, 1, 4, dt);
	matrix_set(A, 1, 7, helper_const);
	
	matrix_set(A, 2, 2, fix16_one);
	matrix_set(A, 2, 5, dt);
	matrix_set(A, 2, 8, helper_const);

	matrix_set(A, 3, 3, fix16_one);
	matrix_set(A, 3, 6, dt);

	matrix_set(A, 4, 4, fix16_one);
	matrix_set(A, 4, 7, dt);

	matrix_set(A, 5, 5, fix16_one);
	matrix_set(A, 5, 8, dt);

	// linear approximation of air resustance
	// F_r = 0.5*rho*c*A*v^2
	// a_r' = (rho/m)*c*A*v
	// air density = 1.225 kg/m^3
	//helper_const = fix16_mul(fix16_div(fix16_from_float(1225.0), m), fix16_mul(c, A1));

	// air resistance
	matrix_set(A, 6, 3, -fix16_from_float(1.614));

	matrix_set(A, 7, 4, -fix16_from_float(1.614));

	matrix_set(A, 8, 5, -fix16_from_float(1.614));


	// get control input model matrix from struct
	mf16 *B = kalman_get_input_transition(&k_pva);

	helper_const = fix16_div(fix16_one, m);
	matrix_set(B, 6, 0, helper_const);
	matrix_set(B, 7, 1, helper_const);
	matrix_set(B, 8, 2, helper_const);

	// get square system state covariance matrix from struct
	mf16 *P = kalman_get_system_covariance(&k_pva);

	matrix_set(P, 0, 0, init_uncert_1);
	matrix_set(P, 1, 1, init_uncert_1);
	matrix_set(P, 2, 2, init_uncert_1);

	matrix_set(P, 3, 3, init_uncert_2);
	matrix_set(P, 4, 4, init_uncert_2);
	matrix_set(P, 5, 5, init_uncert_2);

	matrix_set(P, 6, 6, init_uncert_3);
	matrix_set(P, 7, 7, init_uncert_3);
	matrix_set(P, 8, 8, init_uncert_3);

	// get square contol input covariance matrix from struct
	mf16 *Q = kalman_get_input_covariance(&k_pva);

	matrix_set(Q, 0, 0, sigma);
	matrix_set(Q, 1, 1, sigma);
	matrix_set(Q, 2, 2, sigma);

	// get observation model matrix from struct
	mf16 *H = kalman_get_observation_transformation(&k_pva_m);

	matrix_set(H, 0, 0, fix16_one);

	matrix_set(H, 1, 1, fix16_one);
	
	matrix_set(H, 2, 2, fix16_one);

	matrix_set(H, 3, 3, fix16_one);

	matrix_set(H, 4, 4, fix16_one);

	matrix_set(H, 5, 5, fix16_one);

	matrix_set(H, 6, 6, fix16_one);

	matrix_set(H, 7, 7, fix16_one);

	matrix_set(H, 8, 8, fix16_one);

	// get square observation covariance matrix from struct
	mf16 *R = kalman_get_observation_process_noise(&k_pva_m);		// SET OBSERVATION ERROR

	matrix_set(R, 0, 0, fix16_from_float(0.1));

	matrix_set(R, 1, 1, fix16_from_float(0.1));
	
	matrix_set(R, 2, 2, fix16_from_float(0.01));

	matrix_set(R, 3, 3, fix16_from_float(0.2));

	matrix_set(R, 4, 4, fix16_from_float(0.2));

	matrix_set(R, 5, 5, fix16_from_float(0.1));

	matrix_set(R, 6, 6, fix16_from_float(0.2));

	matrix_set(R, 7, 7, fix16_from_float(0.2));

	matrix_set(R, 8, 8, fix16_from_float(0.2));

	// init timestamp
	last_time = get_sys_time_msec();

	// init u, z vectors and output structure
	update_u();
	update_z();
	update_output();
}

extern void update_output(void) {
	// get state vector from struct
	mf16 *x = kalman_get_state_vector(&k_pva);

	// declaration of variables
	int8_t on_ground = 0;
	float z, z_v;

	// convert position and velocity in z direction to float
	z= POS_FLOAT_OF_BFP(((x->data[2][0]) / (1<<(16-INT32_POS_FRAC))));
	z_v = SPEED_FLOAT_OF_BFP(((x->data[5][0]) * (1<<(INT32_SPEED_FRAC-16))));

	// positions (converted back to paparazzi format)
	kalman_sv_pva.pos_x = (x->data[0][0]) / (1<<(16-INT32_POS_FRAC));
	kalman_sv_pva.pos_y = (x->data[1][0]) / (1<<(16-INT32_POS_FRAC));

	// do not fall below starting position
	if (z < 0.0) {
		kalman_sv_pva.pos_z = 0;
		x->data[2][0] = 0;
		on_ground = 1;
	}
	else {
		kalman_sv_pva.pos_z = (x->data[2][0]) / (1<<(16-INT32_POS_FRAC));
	}

	// velocity (converted back to paparazzi format)
	kalman_sv_pva.vel_x = (x->data[3][0]) * (1<<(INT32_SPEED_FRAC-16));
	kalman_sv_pva.vel_y = (x->data[4][0]) * (1<<(INT32_SPEED_FRAC-16));

	// if already on ground set z-velocity 0:
	if ((on_ground == 1) && (z_v < 0.0)) {
		kalman_sv_pva.vel_z = 0;
		x->data[5][0] = 0;
	}
	else {
		kalman_sv_pva.vel_z = (x->data[5][0]) * (1<<(INT32_SPEED_FRAC-16));
	}

	// acceleration (converted back to paparazzi format)
	kalman_sv_pva.acc_x = (x->data[6][0]) / (1<<(16-INT32_ACCEL_FRAC));
	kalman_sv_pva.acc_y = (x->data[7][0]) / (1<<(16-INT32_ACCEL_FRAC));
	kalman_sv_pva.acc_z = (x->data[8][0]) / (1<<(16-INT32_ACCEL_FRAC));
}

// prediction step (periodic function call by paparazzi)
extern void predict(void) {
	update_u();
	kalman_predict(&k_pva);
	update_output();
}

// correction step (periodic function call by paparazzi)
extern void correct(void) {
	update_z();
	kalman_correct(&k_pva, &k_pva_m);
	update_output();
}

// telemetry
void send_kalman_telemetry(struct transport_tx *trans, struct link_device* link) {
	trans=trans;
	link=link;

	// conversion to float
	float pos_x = POS_FLOAT_OF_BFP(kalman_sv_pva.pos_x);
	float pos_y = POS_FLOAT_OF_BFP(kalman_sv_pva.pos_y);
	float pos_z = POS_FLOAT_OF_BFP(kalman_sv_pva.pos_z);
	float vel_x = SPEED_FLOAT_OF_BFP(kalman_sv_pva.vel_x);
	float vel_y = SPEED_FLOAT_OF_BFP(kalman_sv_pva.vel_y);
	float vel_z = SPEED_FLOAT_OF_BFP(kalman_sv_pva.vel_z);
	float acc_x = ACCEL_FLOAT_OF_BFP(kalman_sv_pva.acc_x);
	float acc_y = ACCEL_FLOAT_OF_BFP(kalman_sv_pva.acc_y);
	float acc_z = ACCEL_FLOAT_OF_BFP(kalman_sv_pva.acc_z);	

	DOWNLINK_SEND_KALMAN(
		DefaultChannel,
		DefaultDevice,
		&pos_x,
		&pos_y,
		&pos_z,
		&vel_x,
		&vel_y,
		&vel_z,
		&acc_x,
		&acc_y,
		&acc_z
	);
}
