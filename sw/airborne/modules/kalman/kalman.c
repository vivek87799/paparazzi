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
#include "modules/finken_model/finken_model_actuators.h"	// source of control parameters for u

int32_t dt_last_measure;
struct state_vector_kalman kalman_sv_pva;

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
}

// update input vector
void update_u(void) {
	mf16 *u = kalman_get_input_vector(&k_pva);

	// get controll data from finken model
	// Roll --> alpha
	// Pitch --> beta
	// Yaw --> theta
	fix16_t alpha_sin = fix16_sin(fix16_from_float(finken_actuators_model.alpha));
	fix16_t alpha_cos = fix16_cos(fix16_from_float(finken_actuators_model.alpha));

	fix16_t beta_sin = fix16_sin(fix16_from_float(finken_actuators_model.beta));
	fix16_t beta_cos = fix16_cos(fix16_from_float(finken_actuators_model.beta));

	fix16_t theta_sin = fix16_sin(fix16_from_float(finken_actuators_model.theta));
	fix16_t theta_cos = fix16_cos(fix16_from_float(finken_actuators_model.theta));

	fix16_t thrust = fix16_from_float(finken_actuators_model.thrust);

	// update input vector
	u->data[0][0] = fix16_mul(fix16_add(fix16_mul(theta_cos, fix16_mul(beta_sin, alpha_cos)), 
		fix16_mul(theta_cos, alpha_sin)), thrust);	// Thrust in X-Direction
    u->data[1][0] = fix16_mul(fix16_sub(fix16_mul(theta_sin, fix16_mul(beta_sin, alpha_cos)), 
		fix16_mul(theta_cos, alpha_sin)), thrust);	// Thrust in Y-Direction
    u->data[2][0] = fix16_mul(fix16_mul(beta_cos, alpha_cos), thrust);	// Thrust in Z-Direction
}

// update observation vector
void update_z(void) {
	mf16 *z = kalman_get_observation_vector(&k_pva_m);

	// ToDo

	z->data[0][0] = 0;	// pos_x
    z->data[1][0] = 0;	// pos_y
    z->data[2][0] = 0;	// pos_z
	z->data[3][0] = 0;	// vel_x
    z->data[4][0] = 0;	// vel_y
    z->data[5][0] = 0;	// vel_z
	z->data[6][0] = 0;	// acc_x
    z->data[7][0] = 0;	// acc_y
    z->data[8][0] = 0;	// acc_z
}

// initialize kalman filter
void kalman_init(void) {
	// time and other constants
	const fix16_t dt = fix16_div(fix16_one,fix16_from_float(30.0));		// SET TIME CONSTANT!!!
	const fix16_t dt_2 = fix16_sq(dt);
	const fix16_t dt_3 = fix16_mul(dt, dt_2);
	const fix16_t dt_4 = fix16_sq(dt_2);
	const fix16_t m = fix16_from_float(1.0);				// SET MASS!!!
	const fix16_t init_uncert = fix16_from_float(1.0);		// SET INITIAL UNCERTEANTY!!!
	const fix16_t sigma = fix16_from_float(1.0);			// SET SIGMA!!!
	fix16_t helper_const;							// varible to speed up matrix assignment

	// init output struct
	kalman_sv_init();

	// init filter
	kalman_filter_initialize(&k_pva, K_NUM_S, K_NUM_I);

	// init observation
	kalman_observation_initialize(&k_pva_m, K_NUM_S, K_NUM_MEAS);

	// get state vector from struct
	mf16 *x = kalman_get_state_vector(&k_pva);

	// set state vector
	x->data[0][0] = 0;	// pos_x
    x->data[1][0] = 0;	// pos_y
    x->data[2][0] = 0;	// pos_z
	x->data[3][0] = 0;	// vel_x
    x->data[4][0] = 0;	// vel_y
    x->data[5][0] = 0;	// vel_z
	x->data[6][0] = 0;	// acc_x
    x->data[7][0] = 0;	// acc_y
    x->data[8][0] = 0;	// acc_z

	// get system state transition model matrix from struct
	mf16 *A = kalman_get_state_transition(&k_pva);

	matrix_set(A, 0, 0, fix16_one);
	matrix_set(A, 0, 1, 0);
	matrix_set(A, 0, 2, 0);
	matrix_set(A, 0, 3, dt);
	matrix_set(A, 0, 4, 0);
	matrix_set(A, 0, 5, 0);
	matrix_set(A, 0, 6, 0);
	matrix_set(A, 0, 7, 0);
	matrix_set(A, 0, 8, 0);

	matrix_set(A, 1, 0, 0);
	matrix_set(A, 1, 1, fix16_one);
	matrix_set(A, 1, 2, 0);
	matrix_set(A, 1, 3, 0);
	matrix_set(A, 1, 4, dt);
	matrix_set(A, 1, 5, 0);
	matrix_set(A, 1, 6, 0);
	matrix_set(A, 1, 7, 0);
	matrix_set(A, 1, 8, 0);
	
	matrix_set(A, 2, 0, 0);
	matrix_set(A, 2, 1, 0);
	matrix_set(A, 2, 2, fix16_one);
	matrix_set(A, 2, 3, 0);
	matrix_set(A, 2, 4, 0);
	matrix_set(A, 2, 5, dt);
	matrix_set(A, 2, 6, 0);
	matrix_set(A, 2, 7, 0);
	matrix_set(A, 2, 8, 0);

	matrix_set(A, 3, 0, 0);
	matrix_set(A, 3, 1, 0);
	matrix_set(A, 3, 2, 0);
	matrix_set(A, 3, 3, fix16_one);
	matrix_set(A, 3, 4, 0);
	matrix_set(A, 3, 5, 0);
	matrix_set(A, 3, 6, 0);
	matrix_set(A, 3, 7, 0);
	matrix_set(A, 3, 8, 0);

	matrix_set(A, 4, 0, 0);
	matrix_set(A, 4, 1, 0);
	matrix_set(A, 4, 2, 0);
	matrix_set(A, 4, 3, 0);
	matrix_set(A, 4, 4, fix16_one);
	matrix_set(A, 4, 5, 0);
	matrix_set(A, 4, 6, 0);
	matrix_set(A, 4, 7, 0);
	matrix_set(A, 4, 8, 0);

	matrix_set(A, 5, 0, 0);
	matrix_set(A, 5, 1, 0);
	matrix_set(A, 5, 2, 0);
	matrix_set(A, 5, 3, 0);
	matrix_set(A, 5, 4, 0);
	matrix_set(A, 5, 5, fix16_one);
	matrix_set(A, 5, 6, 0);
	matrix_set(A, 5, 7, 0);
	matrix_set(A, 5, 8, 0);

	matrix_set(A, 6, 0, 0);
	matrix_set(A, 6, 1, 0);
	matrix_set(A, 6, 2, 0);
	matrix_set(A, 6, 3, 0);
	matrix_set(A, 6, 4, 0);
	matrix_set(A, 6, 5, 0);
	matrix_set(A, 6, 6, 0);
	matrix_set(A, 6, 7, 0);
	matrix_set(A, 6, 8, 0);

	matrix_set(A, 7, 0, 0);
	matrix_set(A, 7, 1, 0);
	matrix_set(A, 7, 2, 0);
	matrix_set(A, 7, 3, 0);
	matrix_set(A, 7, 4, 0);
	matrix_set(A, 7, 5, 0);
	matrix_set(A, 7, 6, 0);
	matrix_set(A, 7, 7, 0);
	matrix_set(A, 7, 8, 0);

	matrix_set(A, 8, 0, 0);
	matrix_set(A, 8, 1, 0);
	matrix_set(A, 8, 2, 0);
	matrix_set(A, 8, 3, 0);
	matrix_set(A, 8, 4, 0);
	matrix_set(A, 8, 5, 0);
	matrix_set(A, 8, 6, 0);
	matrix_set(A, 8, 7, 0);
	matrix_set(A, 8, 8, 0);

	// get control input model matrix from struct
	mf16 *B = kalman_get_input_transition(&k_pva);

	helper_const = fix16_div(dt_2, fix16_mul(2, m));

	matrix_set(B, 0, 0, helper_const);
	matrix_set(B, 0, 1, 0);
	matrix_set(B, 0, 2, 0);

	matrix_set(B, 1, 0, 0);
	matrix_set(B, 1, 1, helper_const);
	matrix_set(B, 1, 2, 0);
	
	matrix_set(B, 2, 0, 0);
	matrix_set(B, 2, 1, 0);
	matrix_set(B, 2, 2, helper_const);

	helper_const = fix16_div(dt, m);

	matrix_set(B, 3, 0, helper_const);
	matrix_set(B, 3, 1, 0);
	matrix_set(B, 3, 2, 0);

	matrix_set(B, 4, 0, 0);
	matrix_set(B, 4, 1, helper_const);
	matrix_set(B, 4, 2, 0);
	
	matrix_set(B, 5, 0, 0);
	matrix_set(B, 5, 1, 0);
	matrix_set(B, 5, 2, helper_const);

	matrix_set(B, 6, 0, fix16_one);
	matrix_set(B, 6, 1, 0);
	matrix_set(B, 6, 2, 0);

	matrix_set(B, 7, 0, 0);
	matrix_set(B, 7, 1, fix16_one);
	matrix_set(B, 7, 2, 0);
	
	matrix_set(B, 8, 0, 0);
	matrix_set(B, 8, 1, 0);
	matrix_set(B, 8, 2, fix16_one);

	// get square system state covariance matrix from struct
	mf16 *P = kalman_get_system_covariance(&k_pva);
	matrix_set(P, 0, 0, 0);
	matrix_set(P, 0, 1, 0);
	matrix_set(P, 0, 2, 0);
	matrix_set(P, 0, 3, 0);
	matrix_set(P, 0, 4, 0);
	matrix_set(P, 0, 5, 0);
	matrix_set(P, 0, 6, 0);
	matrix_set(P, 0, 7, 0);
	matrix_set(P, 0, 8, 0);

	matrix_set(P, 1, 0, 0);
	matrix_set(P, 1, 1, 0);
	matrix_set(P, 1, 2, 0);
	matrix_set(P, 1, 3, 0);
	matrix_set(P, 1, 4, 0);
	matrix_set(P, 1, 5, 0);
	matrix_set(P, 1, 6, 0);
	matrix_set(P, 1, 7, 0);
	matrix_set(P, 1, 8, 0);

	matrix_set(P, 2, 0, 0);
	matrix_set(P, 2, 1, 0);
	matrix_set(P, 2, 2, 0);
	matrix_set(P, 2, 3, 0);
	matrix_set(P, 2, 4, 0);
	matrix_set(P, 2, 5, 0);
	matrix_set(P, 2, 6, 0);
	matrix_set(P, 2, 7, 0);
	matrix_set(P, 2, 8, 0);

	matrix_set(P, 3, 0, 0);
	matrix_set(P, 3, 1, 0);
	matrix_set(P, 3, 2, 0);
	matrix_set(P, 3, 3, 0);
	matrix_set(P, 3, 4, 0);
	matrix_set(P, 3, 5, 0);
	matrix_set(P, 3, 6, 0);
	matrix_set(P, 3, 7, 0);
	matrix_set(P, 3, 8, 0);

	matrix_set(P, 4, 0, 0);
	matrix_set(P, 4, 1, 0);
	matrix_set(P, 4, 2, 0);
	matrix_set(P, 4, 3, 0);
	matrix_set(P, 4, 4, 0);
	matrix_set(P, 4, 5, 0);
	matrix_set(P, 4, 6, 0);
	matrix_set(P, 4, 7, 0);
	matrix_set(P, 4, 8, 0);

	matrix_set(P, 5, 0, 0);
	matrix_set(P, 5, 1, 0);
	matrix_set(P, 5, 2, 0);
	matrix_set(P, 5, 3, 0);
	matrix_set(P, 5, 4, 0);
	matrix_set(P, 5, 5, 0);
	matrix_set(P, 5, 6, 0);
	matrix_set(P, 5, 7, 0);
	matrix_set(P, 5, 8, 0);

	matrix_set(P, 6, 0, 0);
	matrix_set(P, 6, 1, 0);
	matrix_set(P, 6, 2, 0);
	matrix_set(P, 6, 3, 0);
	matrix_set(P, 6, 4, 0);
	matrix_set(P, 6, 5, 0);
	matrix_set(P, 6, 6, init_uncert);
	matrix_set(P, 6, 7, 0);
	matrix_set(P, 6, 8, 0);

	matrix_set(P, 7, 0, 0);
	matrix_set(P, 7, 1, 0);
	matrix_set(P, 7, 2, 0);
	matrix_set(P, 7, 3, 0);
	matrix_set(P, 7, 4, 0);
	matrix_set(P, 7, 5, 0);
	matrix_set(P, 7, 6, 0);
	matrix_set(P, 7, 7, init_uncert);
	matrix_set(P, 7, 8, 0);

	matrix_set(P, 8, 0, 0);
	matrix_set(P, 8, 1, 0);
	matrix_set(P, 8, 2, 0);
	matrix_set(P, 8, 3, 0);
	matrix_set(P, 8, 4, 0);
	matrix_set(P, 8, 5, 0);
	matrix_set(P, 8, 6, 0);
	matrix_set(P, 8, 7, 0);
	matrix_set(P, 8, 8, init_uncert);

	// get square contol input covariance matrix from struct
	mf16 *Q = kalman_get_input_covariance(&k_pva);

	helper_const = fix16_div(fix16_mul(dt_4, sigma), fix16_from_float(4));
	matrix_set(Q, 0, 0, helper_const);
	matrix_set(Q, 1, 1, helper_const);
	matrix_set(Q, 2, 2, helper_const);

	helper_const = fix16_div(fix16_mul(dt_3, sigma), fix16_from_float(2));
	matrix_set(Q, 0, 3, helper_const);
	matrix_set(Q, 1, 4, helper_const);
	matrix_set(Q, 2, 5, helper_const);
	matrix_set(Q, 3, 0, helper_const);
	matrix_set(Q, 4, 1, helper_const);
	matrix_set(Q, 5, 2, helper_const);

	helper_const = fix16_div(fix16_mul(dt_2, sigma), fix16_from_float(2));
	matrix_set(Q, 0, 6, helper_const);
	matrix_set(Q, 1, 7, helper_const);
	matrix_set(Q, 2, 8, helper_const);
	matrix_set(Q, 6, 0, helper_const);
	matrix_set(Q, 7, 1, helper_const);
	matrix_set(Q, 8, 2, helper_const);

	helper_const = fix16_mul(dt_2, sigma);
	matrix_set(Q, 3, 3, helper_const);
	matrix_set(Q, 4, 4, helper_const);
	matrix_set(Q, 5, 5, helper_const);

	helper_const = fix16_mul(dt, sigma);
	matrix_set(Q, 3, 6, helper_const);
	matrix_set(Q, 4, 7, helper_const);
	matrix_set(Q, 5, 8, helper_const);
	matrix_set(Q, 6, 3, helper_const);
	matrix_set(Q, 7, 4, helper_const);
	matrix_set(Q, 8, 5, helper_const);

	matrix_set(Q, 6, 6, sigma);
	matrix_set(Q, 7, 7, sigma);
	matrix_set(Q, 8, 8, sigma);

	matrix_set(Q, 0, 1, 0);
	matrix_set(Q, 0, 2, 0);
	matrix_set(Q, 0, 4, 0);
	matrix_set(Q, 0, 5, 0);
	matrix_set(Q, 0, 7, 0);
	matrix_set(Q, 0, 8, 0);

	matrix_set(Q, 1, 0, 0);
	matrix_set(Q, 1, 2, 0);
	matrix_set(Q, 1, 3, 0);
	matrix_set(Q, 1, 5, 0);
	matrix_set(Q, 1, 6, 0);
	matrix_set(Q, 1, 8, 0);

	matrix_set(Q, 2, 0, 0);
	matrix_set(Q, 2, 1, 0);
	matrix_set(Q, 2, 3, 0);
	matrix_set(Q, 2, 4, 0);
	matrix_set(Q, 2, 6, 0);
	matrix_set(Q, 2, 7, 0);

	matrix_set(Q, 3, 1, 0);
	matrix_set(Q, 3, 2, 0);
	matrix_set(Q, 3, 4, 0);
	matrix_set(Q, 3, 5, 0);
	matrix_set(Q, 3, 7, 0);
	matrix_set(Q, 3, 8, 0);

	matrix_set(Q, 4, 0, 0);
	matrix_set(Q, 4, 2, 0);
	matrix_set(Q, 4, 3, 0);
	matrix_set(Q, 4, 5, 0);
	matrix_set(Q, 4, 6, 0);
	matrix_set(Q, 4, 8, 0);

	matrix_set(Q, 5, 0, 0);
	matrix_set(Q, 5, 1, 0);
	matrix_set(Q, 5, 3, 0);
	matrix_set(Q, 5, 4, 0);
	matrix_set(Q, 5, 6, 0);
	matrix_set(Q, 5, 7, 0);

	matrix_set(Q, 6, 1, 0);
	matrix_set(Q, 6, 2, 0);
	matrix_set(Q, 6, 4, 0);
	matrix_set(Q, 6, 5, 0);
	matrix_set(Q, 6, 7, 0);
	matrix_set(Q, 6, 8, 0);

	matrix_set(Q, 7, 0, 0);
	matrix_set(Q, 7, 2, 0);
	matrix_set(Q, 7, 3, 0);
	matrix_set(Q, 7, 5, 0);
	matrix_set(Q, 7, 6, 0);
	matrix_set(Q, 7, 8, 0);

	matrix_set(Q, 8, 0, 0);
	matrix_set(Q, 8, 1, 0);
	matrix_set(Q, 8, 3, 0);
	matrix_set(Q, 8, 4, 0);
	matrix_set(Q, 8, 6, 0);
	matrix_set(Q, 8, 7, 0);

	// get observation model matrix from struct
	mf16 *H = kalman_get_observation_transformation(&k_pva_m);

    matrix_set(H, 0, 0, fix16_one);
	matrix_set(H, 0, 1, 0);
	matrix_set(H, 0, 2, 0);
	matrix_set(H, 0, 3, 0);
	matrix_set(H, 0, 4, 0);
	matrix_set(H, 0, 5, 0);
	matrix_set(H, 0, 6, 0);
	matrix_set(H, 0, 7, 0);
	matrix_set(H, 0, 8, 0);

	matrix_set(H, 1, 0, 0);
	matrix_set(H, 1, 1, fix16_one);
	matrix_set(H, 1, 2, 0);
	matrix_set(H, 1, 3, 0);
	matrix_set(H, 1, 4, 0);
	matrix_set(H, 1, 5, 0);
	matrix_set(H, 1, 6, 0);
	matrix_set(H, 1, 7, 0);
	matrix_set(H, 1, 8, 0);
	
	matrix_set(H, 2, 0, 0);
	matrix_set(H, 2, 1, 0);
	matrix_set(H, 2, 2, fix16_one);
	matrix_set(H, 2, 3, 0);
	matrix_set(H, 2, 4, 0);
	matrix_set(H, 2, 5, 0);
	matrix_set(H, 2, 6, 0);
	matrix_set(H, 2, 7, 0);
	matrix_set(H, 2, 8, 0);

	matrix_set(H, 3, 0, 0);
	matrix_set(H, 3, 1, 0);
	matrix_set(H, 3, 2, 0);
	matrix_set(H, 3, 3, fix16_one);
	matrix_set(H, 3, 4, 0);
	matrix_set(H, 3, 5, 0);
	matrix_set(H, 3, 6, 0);
	matrix_set(H, 3, 7, 0);
	matrix_set(H, 3, 8, 0);

	matrix_set(H, 4, 0, 0);
	matrix_set(H, 4, 1, 0);
	matrix_set(H, 4, 2, 0);
	matrix_set(H, 4, 3, 0);
	matrix_set(H, 4, 4, fix16_one);
	matrix_set(H, 4, 5, 0);
	matrix_set(H, 4, 6, 0);
	matrix_set(H, 4, 7, 0);
	matrix_set(H, 4, 8, 0);

	matrix_set(H, 5, 0, 0);
	matrix_set(H, 5, 1, 0);
	matrix_set(H, 5, 2, 0);
	matrix_set(H, 5, 3, 0);
	matrix_set(H, 5, 4, 0);
	matrix_set(H, 5, 5, fix16_one);
	matrix_set(H, 5, 6, 0);
	matrix_set(H, 5, 7, 0);
	matrix_set(H, 5, 8, 0);

	matrix_set(H, 6, 0, 0);
	matrix_set(H, 6, 1, 0);
	matrix_set(H, 6, 2, 0);
	matrix_set(H, 6, 3, 0);
	matrix_set(H, 6, 4, 0);
	matrix_set(H, 6, 5, 0);
	matrix_set(H, 6, 6, fix16_one);
	matrix_set(H, 6, 7, 0);
	matrix_set(H, 6, 8, 0);

	matrix_set(H, 7, 0, 0);
	matrix_set(H, 7, 1, 0);
	matrix_set(H, 7, 2, 0);
	matrix_set(H, 7, 3, 0);
	matrix_set(H, 7, 4, 0);
	matrix_set(H, 7, 5, 0);
	matrix_set(H, 7, 6, 0);
	matrix_set(H, 7, 7, fix16_one);
	matrix_set(H, 7, 8, 0);

	matrix_set(H, 8, 0, 0);
	matrix_set(H, 8, 1, 0);
	matrix_set(H, 8, 2, 0);
	matrix_set(H, 8, 3, 0);
	matrix_set(H, 8, 4, 0);
	matrix_set(H, 8, 5, 0);
	matrix_set(H, 8, 6, 0);
	matrix_set(H, 8, 7, 0);
	matrix_set(H, 8, 8, fix16_one);

    // get square observation covariance matrix from struct
    mf16 *R = kalman_get_observation_process_noise(&k_pva_m);		// SET OBSERVATION ERROR

    matrix_set(R, 0, 0, fix16_one);
	matrix_set(R, 0, 1, 0);
	matrix_set(R, 0, 2, 0);
	matrix_set(R, 0, 3, 0);
	matrix_set(R, 0, 4, 0);
	matrix_set(R, 0, 5, 0);
	matrix_set(R, 0, 6, 0);
	matrix_set(R, 0, 7, 0);
	matrix_set(R, 0, 8, 0);

	matrix_set(R, 1, 0, 0);
	matrix_set(R, 1, 1, fix16_one);
	matrix_set(R, 1, 2, 0);
	matrix_set(R, 1, 3, 0);
	matrix_set(R, 1, 4, 0);
	matrix_set(R, 1, 5, 0);
	matrix_set(R, 1, 6, 0);
	matrix_set(R, 1, 7, 0);
	matrix_set(R, 1, 8, 0);
	
	matrix_set(R, 2, 0, 0);
	matrix_set(R, 2, 1, 0);
	matrix_set(R, 2, 2, fix16_one);
	matrix_set(R, 2, 3, 0);
	matrix_set(R, 2, 4, 0);
	matrix_set(R, 2, 5, 0);
	matrix_set(R, 2, 6, 0);
	matrix_set(R, 2, 7, 0);
	matrix_set(R, 2, 8, 0);

	matrix_set(R, 3, 0, 0);
	matrix_set(R, 3, 1, 0);
	matrix_set(R, 3, 2, 0);
	matrix_set(R, 3, 3, fix16_one);
	matrix_set(R, 3, 4, 0);
	matrix_set(R, 3, 5, 0);
	matrix_set(R, 3, 6, 0);
	matrix_set(R, 3, 7, 0);
	matrix_set(R, 3, 8, 0);

	matrix_set(R, 4, 0, 0);
	matrix_set(R, 4, 1, 0);
	matrix_set(R, 4, 2, 0);
	matrix_set(R, 4, 3, 0);
	matrix_set(R, 4, 4, fix16_one);
	matrix_set(R, 4, 5, 0);
	matrix_set(R, 4, 6, 0);
	matrix_set(R, 4, 7, 0);
	matrix_set(R, 4, 8, 0);

	matrix_set(R, 5, 0, 0);
	matrix_set(R, 5, 1, 0);
	matrix_set(R, 5, 2, 0);
	matrix_set(R, 5, 3, 0);
	matrix_set(R, 5, 4, 0);
	matrix_set(R, 5, 5, fix16_one);
	matrix_set(R, 5, 6, 0);
	matrix_set(R, 5, 7, 0);
	matrix_set(R, 5, 8, 0);

	matrix_set(R, 6, 0, 0);
	matrix_set(R, 6, 1, 0);
	matrix_set(R, 6, 2, 0);
	matrix_set(R, 6, 3, 0);
	matrix_set(R, 6, 4, 0);
	matrix_set(R, 6, 5, 0);
	matrix_set(R, 6, 6, fix16_one);
	matrix_set(R, 6, 7, 0);
	matrix_set(R, 6, 8, 0);

	matrix_set(R, 7, 0, 0);
	matrix_set(R, 7, 1, 0);
	matrix_set(R, 7, 2, 0);
	matrix_set(R, 7, 3, 0);
	matrix_set(R, 7, 4, 0);
	matrix_set(R, 7, 5, 0);
	matrix_set(R, 7, 6, 0);
	matrix_set(R, 7, 7, fix16_one);
	matrix_set(R, 7, 8, 0);

	matrix_set(R, 8, 0, 0);
	matrix_set(R, 8, 1, 0);
	matrix_set(R, 8, 2, 0);
	matrix_set(R, 8, 3, 0);
	matrix_set(R, 8, 4, 0);
	matrix_set(R, 8, 5, 0);
	matrix_set(R, 8, 6, 0);
	matrix_set(R, 8, 7, 0);
	matrix_set(R, 8, 8, fix16_one);

	update_u();
	update_z();
}

extern void update_output(void) {
	// get state vector from struct
	mf16 *x = kalman_get_state_vector(&k_pva);

	// positions
	kalman_sv_pva.pos_x = (x->data[0][0]) >> (16-INT32_POS_FRAC);
	kalman_sv_pva.pos_y = (x->data[1][0]) >> (16-INT32_POS_FRAC);
	kalman_sv_pva.pos_z = (x->data[2][0]) >> (16-INT32_POS_FRAC);

	// velocity
	kalman_sv_pva.vel_x = (x->data[3][0]) << (INT32_SPEED_FRAC-16);
	kalman_sv_pva.vel_y = (x->data[4][0]) << (INT32_SPEED_FRAC-16);
	kalman_sv_pva.vel_z = (x->data[5][0]) << (INT32_SPEED_FRAC-16);

	// acceleration
	kalman_sv_pva.acc_x = (x->data[6][0]) >> (16-INT32_ACCEL_FRAC);
	kalman_sv_pva.acc_y = (x->data[7][0]) >> (16-INT32_ACCEL_FRAC);
	kalman_sv_pva.acc_z = (x->data[8][0]) >> (16-INT32_ACCEL_FRAC);
}

// prediction step
extern void predict(void) {
	update_u();
	kalman_predict(&k_pva);
	update_output();
}

// correction step
extern void correct(void) {
	update_z();
	kalman_correct(&k_pva, &k_pva_m);
	update_output();
}
