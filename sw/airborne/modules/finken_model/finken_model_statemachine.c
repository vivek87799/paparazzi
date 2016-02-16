#include "modules/finken_model/finken_model_statemachine.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/electrical.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

bool finken_oscillating_mode;
uint8_t finite_state;
bool search_neighbor;
bool check_direction;

enum sonar_direction{front, side};

static uint16_t getSensorValue(uint16_t ac_id, enum sonar_direction sensor_pos){
    
    if( sensor_pos == front ){
        
        return finken_sensor_model.distance_d_front;
        
    } else {
        
        switch (ac_id) {
                
            case 201: //white
                return finken_sensor_model.distance_d_right;
                
            case 202: //purple
                return finken_sensor_model.distance_d_right;
                
            case 203: //green
                return finken_sensor_model.distance_d_left;
                
            case 204: //blue
                return finken_sensor_model.distance_d_left;
                
            default: return 0;
        }
    }
}


void update_actuators_set_point(void);

void finken_statemachine_model_init(void) {

    finken_oscillating_mode = false;
    finite_state = 0;
    search_neighbor = true;
    check_direction = false;

}

void finken_statemachine_model_periodic(void)
{
    if ( finken_oscillating_mode ) {
    
        switch ( finite_state ){
            case 0: // no copters found
                if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND &&
                     getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                    search_neighbor = false;
                    check_direction = false;
                    finite_state = 2;
                } else {
                    if ( (getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND) ||
                         (getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND)) {
                        search_neighbor = true;
                        check_direction = true;
                        finite_state = 1;
                    } else {
                        search_neighbor = true;
                        check_direction = false;
                        finite_state = 0;
                    }
                }
                break;
            case 1: // one copter found
                if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND &&
                     getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                    search_neighbor = false;
                    check_direction = false;
                    finite_state = 2;
                } else {
                    if ( (getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND) ||
                         (getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND)) {
                        search_neighbor = true;
                        check_direction = false;
                        finite_state = 1;
                    } else {
                        search_neighbor = true;
                        check_direction = false;
                        finite_state = 0;
                    }
                }
                break;
            case 2: // both copters found
                if ( getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND &&
                     getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND) {
                    search_neighbor = false;
                    check_direction = false;
                    finite_state = 2;
                } else {
                    if ( (getSensorValue(AC_ID, front) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, front) < FINKEN_SONAR_UPPER_BOUND) ||
                         (getSensorValue(AC_ID, side) > FINKEN_SONAR_LOWER_BOUND && getSensorValue(AC_ID, side) < FINKEN_SONAR_UPPER_BOUND)) {
                        search_neighbor = true;
                        check_direction = true;
                        finite_state = 1;
                    } else {
                        search_neighbor = true;
                        check_direction = true;
                        finite_state = 0;
                    }
                }
                break;
            default: break;
        }
    }
}
