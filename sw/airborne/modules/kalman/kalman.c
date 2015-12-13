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

// variables to delay start of the Kalman filter
bool kalman_take_off = false;
bool kalman_radio_control = false;

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
#define K_NUM_S 3
#define K_NUM_I 1

// structs of kalman filter (controlled)
kalman16_t k_pva_x;
kalman16_t k_pva_y;
kalman16_t k_pva_z;
kalman16_observation_t k_pva_m_x;
kalman16_observation_t k_pva_m_y;
kalman16_observation_t k_pva_m_z;

// create the measurement structure
#define K_MEAS_NAME position
#define K_NUM_MEAS 3

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

// update input vector
void update_u(void) {
	mf16 *u_x = kalman_get_input_vector(&k_pva_x);
	mf16 *u_y = kalman_get_input_vector(&k_pva_y);
	mf16 *u_z = kalman_get_input_vector(&k_pva_z);

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
	// start Kalman filter after copter init
	kalman_radio_control = true;
	
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
	u_x->data[0][0] = fix16_mul(fix16_mul(beta_sin, fix16_from_float(-1.0)), thrust_converted);	// Thrust in X-Direction	
	u_y->data[0][0] = fix16_mul(fix16_mul(alpha_sin, beta_cos), thrust_converted);	// Thrust in Y-Direction
	u_z->data[0][0] = fix16_sub(fix16_mul(m, fix16_from_float(9.81)), 
		fix16_mul(fix16_mul(beta_cos, alpha_cos), thrust_converted));	// Thrust in Z-Direction
}

// update observation vector
void update_z(void) {
	// get observation vector
	mf16 *z_x = kalman_get_observation_vector(&k_pva_m_x);
	mf16 *z_y = kalman_get_observation_vector(&k_pva_m_y);
	mf16 *z_z = kalman_get_observation_vector(&k_pva_m_z);

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

	// handle dead zones
	if (alpha<0.017453 && alpha>-0.017453) {
		alpha = 0;
	}
	if (beta<0.017453 && beta>-0.017453) {
		beta = 0;
	}

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

	// compute system variables from flow sensor in z direction
	position_z = fix16_mul(fix16_mul(R33, position_z), fix16_from_float(-1.0));
	fix16_t velocity_z = fix16_div(fix16_sub(position_z, z_z->data[0][0]), diff);

	// compute current velocity in x and y direction
	helper_const = velocity_x;
	velocity_x = fix16_add(fix16_mul(R11, velocity_x), fix16_mul(R12, velocity_y));
	velocity_y = fix16_add(fix16_mul(R21, helper_const), fix16_mul(R22, velocity_y));

	// indirect computation of position difference to last time in x and y 
	fix16_t position_x = fix16_mul(fix16_div(fix16_add(velocity_x, z_x->data[1][0]), 
		fix16_from_float(2.0)), diff);
	fix16_t position_y = fix16_mul(fix16_div(fix16_add(velocity_y, z_y->data[1][0]), 
		fix16_from_float(2.0)), diff);

	// get accelerometer data
	fix16_t acceleration_x = fix16_from_float(ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.x));
	fix16_t acceleration_y = fix16_from_float(ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.y));
	fix16_t acceleration_z = fix16_from_float(ACCEL_FLOAT_OF_BFP(finken_sensor_model.acceleration.z));

	// update measurement vector vector
	z_x->data[0][0] = fix16_add(z_x->data[0][0], position_x);	  	// pos_x
	z_y->data[0][0] = fix16_add(z_y->data[0][0], position_y);		// pos_y
	z_z->data[0][0] = position_z;		// pos_z
	z_x->data[1][0] = velocity_x;		// vel_x
	z_y->data[1][0] = velocity_y;		// vel_y
	z_z->data[1][0] = velocity_z;		// vel_z
	z_x->data[2][0] = fix16_add(fix16_add(fix16_mul(R11, acceleration_x),
		fix16_mul(R12, acceleration_y)), fix16_mul(R13, acceleration_z));	// acc_x
	z_y->data[2][0] = fix16_add(fix16_add(fix16_mul(R21, acceleration_x), 
		fix16_mul(R22, acceleration_y)), fix16_mul(R23, acceleration_z));	// acc_y
	z_z->data[2][0] = fix16_sub(fix16_add(fix16_add(fix16_mul(R31, acceleration_x), 
		fix16_mul(R32, acceleration_y)), fix16_mul(R33, acceleration_z)),
		fix16_from_float(9.81));	// acc_z

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
	
	// init output struct
	kalman_sv_init();

	// init filter
	kalman_filter_initialize(&k_pva_x, K_NUM_S, K_NUM_I);
	kalman_filter_initialize(&k_pva_y, K_NUM_S, K_NUM_I);
	kalman_filter_initialize(&k_pva_z, K_NUM_S, K_NUM_I);

	// init observation
	kalman_observation_initialize(&k_pva_m_x, K_NUM_S, K_NUM_MEAS);
	kalman_observation_initialize(&k_pva_m_y, K_NUM_S, K_NUM_MEAS);
	kalman_observation_initialize(&k_pva_m_z, K_NUM_S, K_NUM_MEAS);

	// (dt^2)/2
	helper_const = fix16_div(dt_2, fix16_from_float(2.0));

	// get state vector from struct
	//mf16 *x = kalman_get_state_vector(&k_pva);

	// get system state transition model matrix from struct
	mf16 *A_x = kalman_get_state_transition(&k_pva_x);
	mf16 *A_y = kalman_get_state_transition(&k_pva_y);
	mf16 *A_z = kalman_get_state_transition(&k_pva_z);
	
	// s_i(t) = s_i(t-1) + v_i(t-1)*dt + [(1/m)*F_i(t-1)*(dt^2)/2]
	// third term in matrix B
	matrix_set(A_x, 0, 0, fix16_one);
	matrix_set(A_x, 0, 1, dt);

	matrix_set(A_y, 0, 0, fix16_one);
	matrix_set(A_y, 0, 1, dt);
	
	matrix_set(A_z, 0, 0, fix16_one);
	matrix_set(A_z, 0, 1, dt);

	// v_i(t) = v_i(t-1) + [(1/m)*F_i(t-1)*dt]
	// second term in matrix B
	matrix_set(A_x, 1, 1, fix16_one);

	matrix_set(A_y, 1, 1, fix16_one);

	matrix_set(A_z, 1, 1, fix16_one);


	// get control input model matrix from struct
	mf16 *B_x = kalman_get_input_transition(&k_pva_x);
	mf16 *B_y = kalman_get_input_transition(&k_pva_y);
	mf16 *B_z = kalman_get_input_transition(&k_pva_z);

	// s_i(t) = [s_i(t-1) + v_i(t-1)*dt] + (1/m)*F_i(t-1)*(dt^2)/2
	// first two terms in matrix A
	helper_const = fix16_div(fix16_div(dt_2, fix16_from_float(2.0)), m);
	matrix_set(B_x, 0, 0, helper_const);
	matrix_set(B_y, 0, 0, helper_const);
	matrix_set(B_z, 0, 0, helper_const);

	// v_i(t) = [v_i(t-1)] + (1/m)*F_i(t-1)*dt
	// first term in matrix A
	helper_const = fix16_div(dt, m);
	matrix_set(B_x, 1, 0, helper_const);
	matrix_set(B_y, 1, 0, helper_const);
	matrix_set(B_z, 1, 0, helper_const);

	// a_i(t) = (1/m)*F_i(t-1)
	helper_const = fix16_div(fix16_one, m);
	matrix_set(B_x, 2, 0, helper_const);
	matrix_set(B_y, 2, 0, helper_const);
	matrix_set(B_z, 2, 0, helper_const);

	// get square system state covariance matrix from struct
	mf16 *P_x = kalman_get_system_covariance(&k_pva_x);
	mf16 *P_y = kalman_get_system_covariance(&k_pva_y);
	mf16 *P_z = kalman_get_system_covariance(&k_pva_z);

	// prediction covarience initialization
	matrix_set(P_x, 0, 0, fix16_from_float(0.1));
	matrix_set(P_y, 0, 0, fix16_from_float(0.1));
	matrix_set(P_z, 0, 0, fix16_from_float(0.1));

	matrix_set(P_x, 1, 1, fix16_from_float(0.1));
	matrix_set(P_y, 1, 1, fix16_from_float(0.1));
	matrix_set(P_z, 1, 1, fix16_from_float(0.1));

	matrix_set(P_x, 2, 2, fix16_from_float(1.0));
	matrix_set(P_y, 2, 2, fix16_from_float(1.0));
	matrix_set(P_z, 2, 2, fix16_from_float(1.0));

	// get square control input covariance matrix from struct
	mf16 *Q_x = kalman_get_input_covariance(&k_pva_x);
	mf16 *Q_y = kalman_get_input_covariance(&k_pva_y);
	mf16 *Q_z = kalman_get_input_covariance(&k_pva_z);

	// prediction uncertainty
	matrix_set(Q_x, 0, 0, fix16_from_float(0.05));
	matrix_set(Q_y, 0, 0, fix16_from_float(0.05));
	matrix_set(Q_z, 0, 0, fix16_from_float(1.0));

	matrix_set(Q_x, 1, 1, fix16_from_float(0.75));
	matrix_set(Q_y, 1, 1, fix16_from_float(0.75));
	matrix_set(Q_z, 1, 1, fix16_from_float(5.00));

	matrix_set(Q_x, 2, 2, fix16_from_float(1.0));
	matrix_set(Q_y, 2, 2, fix16_from_float(1.0));
	matrix_set(Q_z, 2, 2, fix16_from_float(2.0));

	// get observation model matrix from struct
	mf16 *H_x = kalman_get_observation_transformation(&k_pva_m_x);
	mf16 *H_y = kalman_get_observation_transformation(&k_pva_m_y);
	mf16 *H_z = kalman_get_observation_transformation(&k_pva_m_z);

	// H = I, because we measure everything independently 
	matrix_set(H_x, 0, 0, fix16_one);
	matrix_set(H_y, 0, 0, fix16_one);	
	matrix_set(H_z, 0, 0, fix16_one);

	matrix_set(H_x, 1, 1, fix16_one);
	matrix_set(H_y, 1, 1, fix16_one);
	matrix_set(H_z, 1, 1, fix16_one);

	matrix_set(H_x, 2, 2, fix16_one);
	matrix_set(H_y, 2, 2, fix16_one);
	matrix_set(H_z, 2, 2, fix16_one);

	// get square observation covariance matrix from struct
	mf16 *R_x = kalman_get_observation_process_noise(&k_pva_m_x);
	mf16 *R_y = kalman_get_observation_process_noise(&k_pva_m_y);
	mf16 *R_z = kalman_get_observation_process_noise(&k_pva_m_z);

	// sensor uncertainty in the diagonal of the matrix
	matrix_set(R_x, 0, 0, fix16_from_float(0.05));
	matrix_set(R_y, 0, 0, fix16_from_float(0.05));	
	matrix_set(R_z, 0, 0, fix16_from_float(0.1));

	matrix_set(R_x, 1, 1, fix16_from_float(0.65));
	matrix_set(R_y, 1, 1, fix16_from_float(0.65));
	matrix_set(R_z, 1, 1, fix16_from_float(0.5));

	matrix_set(R_x, 2, 2, fix16_from_float(3.0));
	matrix_set(R_y, 2, 2, fix16_from_float(3.0));
	matrix_set(R_z, 2, 2, fix16_from_float(3.5));

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
	mf16 *x_x = kalman_get_state_vector(&k_pva_x);
	mf16 *x_y = kalman_get_state_vector(&k_pva_y);
	mf16 *x_z = kalman_get_state_vector(&k_pva_z);

	// positions (converted back to paparazzi format)
	kalman_sv_pva.pos_x = POS_BFP_OF_REAL(fix16_to_float(x_x->data[0][0]));
	kalman_sv_pva.pos_y = POS_BFP_OF_REAL(fix16_to_float(x_y->data[0][0]));
	kalman_sv_pva.pos_z = POS_BFP_OF_REAL(fix16_to_float(x_z->data[0][0]));

	// velocity (converted back to paparazzi format)
	kalman_sv_pva.vel_x = SPEED_BFP_OF_REAL(fix16_to_float(x_x->data[1][0]));
	kalman_sv_pva.vel_y = SPEED_BFP_OF_REAL(fix16_to_float(x_y->data[1][0]));
	kalman_sv_pva.vel_z = SPEED_BFP_OF_REAL(fix16_to_float(x_z->data[1][0]));

	// acceleration (converted back to paparazzi format)
	kalman_sv_pva.acc_x = ACCEL_BFP_OF_REAL(fix16_to_float(x_x->data[2][0]));
	kalman_sv_pva.acc_y = ACCEL_BFP_OF_REAL(fix16_to_float(x_y->data[2][0]));
	kalman_sv_pva.acc_z = ACCEL_BFP_OF_REAL(fix16_to_float(x_z->data[2][0]));

}

// prediction step (periodic function call by paparazzi)
extern void predict(void) {
	// runs only if filter is already activated
	if(kalman_take_off || kalman_radio_control) {
		update_u();
		kalman_predict(&k_pva_x);
		kalman_predict(&k_pva_y);
		kalman_predict(&k_pva_z);
	}
}

// correction step (periodic function call by paparazzi)
extern void correct(void) {
	// runs only if filter is already activated
	if(kalman_take_off || kalman_radio_control) {
		update_z();
		kalman_correct(&k_pva_x, &k_pva_m_x);
		kalman_correct(&k_pva_y, &k_pva_m_y);
		kalman_correct(&k_pva_z, &k_pva_m_z);
		update_output();
	}
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
