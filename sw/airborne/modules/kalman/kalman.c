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

float q1;
float q2;

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

// update input vector
void update_u(void) {
	mf16 *u = kalman_get_input_vector(&k_pva);

	// get controll data from finken model
	// Roll --> alpha
	// Pitch --> beta
	// Yaw --> theta

	// ------------------------------------------------------------------------
	// ------------------- start of autopilot as control unit -----------------
	// ------------------------------------------------------------------------

	// controll data from autopilot
	float alpha = (finken_actuators_model.roll / 180.0) * PI;
	float beta = (finken_actuators_model.pitch / 180.0) * PI;
	// float theta = finken_actuators_model.yaw; // [rad/s] winkelgeschwindigkeit

	// theta is permanently set to 0
	float theta = 0.0;

	// keep compiler happy
	float throttle;

	// waiting for heigth controller
	if(finken_system_model_control_height==0) {
		throttle = 0.0;
	}
	else {
		throttle = finken_actuators_set_point.thrust * 100;
	}

	// ------------------------------------------------------------------------
	// -------------------- end of autopilot as control unit ------------------
	// ------------------------------------------------------------------------

	// ------------------------------------------------------------------------
	// ---------------- start of radio controller as control unit -------------
	// ------------------------------------------------------------------------
/*
	// controll data from controller
	float alpha = ((float) radio_control.values[RADIO_ROLL]) / (12236*180) * 20 * PI;
	float beta = ((float) radio_control.values[RADIO_PITCH]) / (12236*180) * 20 * PI;
	//float theta = ((float) radio_control.values[RADIO_YAW]) / (12236*180) * 90 * PI;

	// theta is permanently set to 0
	//float theta = 0.0;

	// correction to 0 with 1318
	float throttle = (((float)radio_control.values[RADIO_THROTTLE]) + 1318.0) / 12236.0 * 100;
	q1 = (float)radio_control.values[RADIO_THROTTLE];

	// handle controller spezified dead zones
	if (alpha<0.017453 && alpha>-0.017453) {
		alpha = 0.0;
	}
	if (beta<0.017453 && beta>-0.017453) {
		beta = 0.0;
	}
*/
	// ------------------------------------------------------------------------
	// ----------------- end of radio controller as control unit --------------
	// ------------------------------------------------------------------------

	// trigonometric variables to reduce future computations
	fix16_t alpha_sin = fix16_sin(fix16_from_float(alpha));
	fix16_t alpha_cos = fix16_cos(fix16_from_float(alpha));

	fix16_t beta_sin = fix16_sin(fix16_from_float(beta));
	fix16_t beta_cos = fix16_cos(fix16_from_float(beta));

	//fix16_t theta_sin = fix16_sin(fix16_from_float(theta));
	//fix16_t theta_cos = fix16_cos(fix16_from_float(theta));

	fix16_t thrust = fix16_from_float(throttle);

	// Conversionsfunction from Christoph: thrust[g] = 0.01514x^2 + 0.65268x [% --> gramm] corresponding to 4 motors converted to kg
	fix16_t thrust_converted = fix16_div(fix16_mul(fix16_add(fix16_mul(fix16_from_float(0.01514), fix16_mul(thrust, thrust)),
		fix16_mul(fix16_from_float(0.65268), thrust)), fix16_from_float(4.0)), fix16_from_float(1000.0));

	// convert to force
	thrust_converted = fix16_mul(thrust_converted, fix16_from_float(9.81));

	// decompore thrust vector to global acceleration vectors and update input vector
	u->data[0][0] = fix16_mul(fix16_mul(beta_sin, fix16_from_float(-1.0)), thrust_converted);	// Thrust in X-Direction	
	u->data[1][0] = fix16_mul(fix16_mul(alpha_sin, beta_cos), thrust_converted);	// Thrust in Y-Direction
	u->data[2][0] = fix16_sub(fix16_mul(m, fix16_from_float(9.81)), 
		fix16_mul(fix16_mul(beta_cos, alpha_cos), thrust_converted));	// Thrust in Z-Direction
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
	fix16_t alpha = fix16_from_float(ANGLE_FLOAT_OF_BFP(finken_sensor_attitude.phi));
	fix16_t beta = fix16_from_float(ANGLE_FLOAT_OF_BFP(finken_sensor_attitude.theta));
	//fix16_t theta = fix16_from_float(ANGLE_FLOAT_OF_BFP(finken_sensor_attitude.psi));

	q1 = fix16_to_float(alpha);
	q2 = fix16_to_float(beta);

	// theta is permanently set to 0
	fix16_t theta = 0;

	// Trigonometric variables to reduce future computation
	fix16_t alpha_sin = fix16_sin(alpha);
	fix16_t alpha_cos = fix16_cos(alpha);

	fix16_t beta_sin = fix16_sin(beta);
	fix16_t beta_cos = fix16_cos(beta);

	fix16_t theta_sin = fix16_sin(theta);
	fix16_t theta_cos = fix16_cos(theta);

	// Rotationsmatrix
	fix16_t R11 = fix16_add(theta_cos, beta_cos);
	fix16_t R12 = fix16_add(theta_sin, beta_cos);
	fix16_t R13 = fix16_mul(beta_sin, fix16_from_float(-1.0));

	fix16_t R21 = fix16_sub(fix16_mul(fix16_mul(theta_cos, beta_sin), alpha_sin), 
		fix16_mul(theta_sin, alpha_cos));
	fix16_t R22 = fix16_add(fix16_mul(fix16_mul(theta_sin, beta_sin), alpha_sin), 
		fix16_mul(theta_cos, alpha_cos));
	fix16_t R23 = fix16_mul(beta_cos, alpha_sin);

	fix16_t R31 = fix16_add(fix16_mul(fix16_mul(theta_cos, beta_sin), alpha_cos), 
		fix16_mul(theta_sin, alpha_sin));
	fix16_t R32 = fix16_sub(fix16_mul(fix16_mul(theta_sin, beta_sin), alpha_cos), 
		fix16_mul(theta_cos, alpha_sin));
	fix16_t R33 = fix16_mul(beta_cos, alpha_cos);

	// get data of flow sensor
	fix16_t velocity_x = fix16_from_float(SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.x));
	fix16_t velocity_y = fix16_from_float(SPEED_FLOAT_OF_BFP(finken_sensor_model.velocity.y));
	fix16_t position_z = fix16_from_float(POS_FLOAT_OF_BFP(finken_sensor_model.pos.z));

	// scaling because of flow sensor (16/4.6)
	//velocity_x = fix16_mul(velocity_x, fix16_from_float(3.47826087));
	//velocity_y = fix16_mul(velocity_y, fix16_from_float(3.47826087));

	// compute system variables from flow sensor in z direction
	position_z = fix16_mul(fix16_mul(R33, position_z), fix16_from_float(-1.0));
	fix16_t velocity_z = fix16_div(fix16_sub(position_z, z->data[2][0]), diff);

	// compute current velocity in x and y direction
	helper_const = velocity_x;
	velocity_x = fix16_add(fix16_mul(R11, velocity_x), fix16_mul(R12, velocity_y));
	velocity_y = fix16_add(fix16_mul(R21, helper_const), fix16_mul(R22, velocity_y));

	// indirect computation of position difference to last time in x and y 
	fix16_t position_x = fix16_mul(fix16_div(fix16_add(velocity_x, z->data[3][0]), 
		fix16_from_float(2.0)), diff);
	fix16_t position_y = fix16_mul(fix16_div(fix16_add(velocity_y, z->data[4][0]), 
		fix16_from_float(2.0)), diff);

	// get accelerometer data
	fix16_t acceleration_x = fix16_from_float(ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.x));
	fix16_t acceleration_y = fix16_from_float(ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.y));
	fix16_t acceleration_z = fix16_from_float(ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.z));

	// update measurement vector vector
	z->data[0][0] = fix16_add(z->data[0][0], position_x);	  	// pos_x
	z->data[1][0] = fix16_add(z->data[1][0], position_y);		// pos_y
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
	const fix16_t dt = fix16_div(fix16_one,fix16_from_float(KALMAN_PREDICTION_FREQ));		// time constant (paparazzi call frequency)
	const fix16_t dt_2 = fix16_sq(dt);

	// initialisation constants
	m = fix16_from_float(KALMAN_MASS);						// set mass [kg]
	fix16_t helper_const;									// varible to speed up matrix assignment

	// this constants were declared to calculate the air resistance
	// but they were removed because of linearisation
	// but they can be useful in the future
	// ------------------------------------------------------------------------ 
	//const fix16_t c = fix16_from_float(0.95);				// c = 0.95 --> ~ as it would be a plane surface
	//const fix16_t A1 = fix16_from_float(0.03947);			// A_x,y = 27.5*27.5 and 12*27,5 rotated with 5 degrees ~ 0.03947
	//const fix16_t A2 = fix16_from_float(0.075625);		// A_z = 27.5*27.5 cm --> maximal distance between endpoints of the rotors
	// ------------------------------------------------------------------------
	
	// init output struct
	kalman_sv_init();

	// init filter
	kalman_filter_initialize(&k_pva, K_NUM_S, K_NUM_I);

	// init observation
	kalman_observation_initialize(&k_pva_m, K_NUM_S, K_NUM_MEAS);

	// (dt^2)/2
	helper_const = fix16_div(dt_2, fix16_from_float(2.0));

	// get state vector from struct
	mf16 *x = kalman_get_state_vector(&k_pva);

	// initialize acceleration in z direction
	matrix_set(x, 8, 0, fix16_from_float(9.81));

	// get system state transition model matrix from struct
	mf16 *A = kalman_get_state_transition(&k_pva);
	
	// s_i(t) = s_i(t-1) + v_i(t-1)*dt + a_i(t-1)*(dt^2)/2
	matrix_set(A, 0, 0, fix16_one);
	matrix_set(A, 0, 3, dt);

	matrix_set(A, 1, 1, fix16_one);
	matrix_set(A, 1, 4, dt);
	
	matrix_set(A, 2, 2, fix16_one);
	matrix_set(A, 2, 5, dt);

	// v_i(t) = v_i(t-1) + a_i(t-1)*dt
	matrix_set(A, 3, 3, fix16_one);

	matrix_set(A, 4, 4, fix16_one);

	matrix_set(A, 5, 5, fix16_one);

	// this linearistaion was not working (can be useful in the future)
	// ------------------------------------------------------------------------
	// linear approximation of air resistance
	// F_r = 0.5*rho*c*A*v^2
	// a_r' = (rho/m)*c*A*v
	// air density = 1.225 kg/m^3
	// helper_const = fix16_mul(fix16_div(fix16_from_float(1225.0), m), fix16_mul(c, A1));
	// ------------------------------------------------------------------------

	// air resistance
	// linearised using 0 and maximal allowed speed to a given mass
	// a_i(t) = -c*v(t-1) + [1/m*F_i(t-1)]
	// second part is in the B matrix
	//matrix_set(A, 6, 3, fix16_from_float(-1.614));

	//matrix_set(A, 7, 4, fix16_from_float(-1.614));

	//matrix_set(A, 8, 5, fix16_from_float(-1.614));


	// get control input model matrix from struct
	mf16 *B = kalman_get_input_transition(&k_pva);

	helper_const = fix16_div(fix16_div(dt_2, fix16_from_float(2.0)), m);
	matrix_set(B, 0, 0, helper_const);
	matrix_set(B, 1, 1, helper_const);
	matrix_set(B, 2, 2, helper_const);

	helper_const = fix16_div(dt, m);
	matrix_set(B, 3, 0, helper_const);
	matrix_set(B, 4, 1, helper_const);
	matrix_set(B, 5, 2, helper_const);

	// a_i(t) = [-c*v(t-1)] + 1/m*F_i(t-1)
	// first part is in the A matrix
	helper_const = fix16_div(fix16_one, m);
	matrix_set(B, 6, 0, helper_const);
	matrix_set(B, 7, 1, helper_const);
	matrix_set(B, 8, 2, helper_const);

	// get square system state covariance matrix from struct
	mf16 *P = kalman_get_system_covariance(&k_pva);

	// prediction covarience
	matrix_set(P, 0, 0, fix16_from_float(0.01));
	matrix_set(P, 1, 1, fix16_from_float(0.01));
	matrix_set(P, 2, 2, fix16_from_float(0.01));

	matrix_set(P, 3, 3, fix16_from_float(0.01));
	matrix_set(P, 4, 4, fix16_from_float(0.01));
	matrix_set(P, 5, 5, fix16_from_float(0.01));

	matrix_set(P, 6, 6, fix16_from_float(0.01));
	matrix_set(P, 7, 7, fix16_from_float(0.01));
	matrix_set(P, 8, 8, fix16_from_float(0.1));

	// get square control input covariance matrix from struct
	mf16 *Q = kalman_get_input_covariance(&k_pva);

	// Q is defined in most of the literature as Q = B S (B^T)
	// where S contains the noise
	// here Q = S
	matrix_set(Q, 0, 0, fix16_from_float(0.5));
	matrix_set(Q, 1, 1, fix16_from_float(0.5));
	matrix_set(Q, 2, 2, fix16_from_float(1.0));
	matrix_set(Q, 3, 3, fix16_from_float(0.5));
	matrix_set(Q, 4, 4, fix16_from_float(0.5));
	matrix_set(Q, 5, 5, fix16_from_float(1.0));
	matrix_set(Q, 6, 6, fix16_from_float(1.0));
	matrix_set(Q, 7, 7, fix16_from_float(1.0));
	matrix_set(Q, 8, 8, fix16_from_float(1.0));

	// get observation model matrix from struct
	mf16 *H = kalman_get_observation_transformation(&k_pva_m);

	// H = I, because we measure everything independently 
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
	mf16 *R = kalman_get_observation_process_noise(&k_pva_m);		// set observation error

	// sensor uncertainty in the diagonal of the matrix
	matrix_set(R, 0, 0, fix16_from_float(0.05));
	matrix_set(R, 1, 1, fix16_from_float(0.05));	
	matrix_set(R, 2, 2, fix16_from_float(0.01));
	matrix_set(R, 3, 3, fix16_from_float(0.3));
	matrix_set(R, 4, 4, fix16_from_float(0.3));
	matrix_set(R, 5, 5, fix16_from_float(0.15));
	matrix_set(R, 6, 6, fix16_from_float(3.0));
	matrix_set(R, 7, 7, fix16_from_float(3.0));
	matrix_set(R, 8, 8, fix16_from_float(3.0));

	// init timestamp
	last_time = get_sys_time_msec();

	// init u, z vectors and output structure
	update_u();
	update_z();
	update_output();
}

// update output structure
extern void update_output(void) {
	// get state vector from struct
	mf16 *x = kalman_get_state_vector(&k_pva);

	// positions (converted back to paparazzi format)
	kalman_sv_pva.pos_x = POS_BFP_OF_REAL(fix16_to_float(x->data[0][0]));
	kalman_sv_pva.pos_y = POS_BFP_OF_REAL(fix16_to_float(x->data[1][0]));
	kalman_sv_pva.pos_z = POS_BFP_OF_REAL(fix16_to_float(x->data[2][0]));

	// velocity (converted back to paparazzi format)
	kalman_sv_pva.vel_x = SPEED_BFP_OF_REAL(fix16_to_float(x->data[3][0]));
	kalman_sv_pva.vel_y = SPEED_BFP_OF_REAL(fix16_to_float(x->data[4][0]));
	kalman_sv_pva.vel_z = SPEED_BFP_OF_REAL(fix16_to_float(x->data[5][0]));

	// acceleration (converted back to paparazzi format)
	kalman_sv_pva.acc_x = ACCEL_BFP_OF_REAL(fix16_to_float(x->data[6][0]));
	kalman_sv_pva.acc_y = ACCEL_BFP_OF_REAL(fix16_to_float(x->data[7][0]));
	kalman_sv_pva.acc_z = ACCEL_BFP_OF_REAL(fix16_to_float(x->data[8][0]));

}

// prediction step (periodic function call by paparazzi)
extern void predict(void) {
	update_u();
	kalman_predict(&k_pva);
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
