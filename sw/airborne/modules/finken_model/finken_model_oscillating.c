#include "modules/finken_model/finken_model_oscillating.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/electrical.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>
#include "math/pprz_algebra_int.h"

float height_oscillating_down;
float height_oscillating_up;
float middle;
float height_changing_rate;
bool go_down;


void update_actuators_set_point(void);

void finken_oscillating_model_init(void) {

    height_oscillating_up = 0.9;
    height_oscillating_down = 0.40;
    middle = ((height_oscillating_up - height_oscillating_down) / 2) + height_oscillating_down;
    // periodic function is called 5 times every second
    // which results in height changing 2,5 cm per second
    height_changing_rate = 0.005;
    go_down = false;
}

void finken_oscillating_model_periodic(void)
{
    if ( finken_oscillating_mode ) {
        
        if( search_neighbor ){
            
            // check weather go up or down
            if ( check_direction == true ) {
                
                // if we are above middle, go downwards and otherwise
                if ( POS_FLOAT_OF_BFP(finken_sensor_model.pos.z) >= middle ) {
                    go_down = true;
                } else {
                    go_down = false;
                }
                check_direction = false;
            }
        
            if ( go_down ){
                if ( finken_system_set_point.z > height_oscillating_down ){
                    finken_system_set_point.z -= height_changing_rate;
                } else {
                    finken_system_set_point.z = height_oscillating_down;
                    go_down = false;
                }
            
            } else {
                if ( finken_system_set_point.z < height_oscillating_up ){
                    finken_system_set_point.z += height_changing_rate;
                } else {
                    finken_system_set_point.z = height_oscillating_up;
                    go_down = true;
                }
            }
        }
    }
}
