#include "modules/finken_model/finken_model_oscillating.h"
#include "subsystems/navigation/common_flight_plan.h"
#include "subsystems/electrical.h"

/* input */
#include "modules/finken_model/finken_model_sensors.h"
#include "modules/finken_model/finken_model_environment.h"


#include "firmwares/rotorcraft/autopilot.h"
#include <math.h>

//struct system_model_s finken_oscillating_model;

uint16_t finken_oscillating_last_time;

bool finken_oscillating_mode;
float height_oscillating_down;
float height_oscillating_up;
float time_oscillating;
float height_changing_rate;
bool go_down;
bool search_neighbor;


void update_actuators_set_point(void);

void finken_oscillating_model_init(void) {
//    finken_oscillating_model.distance_z     = 0.0;
//    finken_oscillating_model.velocity_theta = 0.0;
//    finken_oscillating_model.velocity_x     = 0.0;
//    finken_oscillating_model.velocity_y     = 0.0;
//
//    finken_actuators_set_point.alpha  = 0.0;
//	finken_actuators_set_point.beta   = 0.0;
//	finken_actuators_set_point.theta  = 0.0;
//	finken_actuators_set_point.thrust = 0.0;
//    
//    register_periodic_telemetry(DefaultPeriodic, "FINKEN_SYSTEM_MODEL", send_finken_oscillating_model_telemetry);

    
    height_oscillating_down = 0.40;
    height_oscillating_up = 0.9;
    finken_oscillating_last_time = 0;
    height_changing_rate = 0.05;
    go_down = false;
    finken_oscillating_mode = false;
    search_neighbor = true;

}

void finken_oscillating_model_periodic(void)
{
 
//    update_finken_oscillating_model();
    
    
    if ( finken_oscillating_mode ) {

	switch ( AC_ID ){
                    case 202:   //purple
                                if ( finken_sensor_model.distance_d_back > 30 && finken_sensor_model.distance_d_back < 60 ){
					search_neighbor = false;
				} else {
					if ( finken_sensor_model.distance_d_right > 30 && finken_sensor_model.distance_d_right < 60 ){
                                    		search_neighbor = false;
                                	} else {
						search_neighbor = true;
					}
				}
                        break;
                    case 203:   //green
                                if ( finken_sensor_model.distance_d_back > 30 && finken_sensor_model.distance_d_back < 60 ){
					search_neighbor = false;
				} else {
					if ( finken_sensor_model.distance_d_left > 30 && finken_sensor_model.distance_d_left < 60 ){
                                    		search_neighbor = false;
                                	} else {
						search_neighbor = true;
					}
				}
                        break;
		    case 201:   //white
                                if ( finken_sensor_model.distance_d_back > 30 && finken_sensor_model.distance_d_back < 60 ){
					search_neighbor = false;
				} else {
					if ( finken_sensor_model.distance_d_left > 30 && finken_sensor_model.distance_d_left < 60 ){
                                    		search_neighbor = false;
                                	} else {
						search_neighbor = true;
					}
				}
                        break;
		    default: break;
                }

        if ( (finken_oscillating_last_time + 1) <= stage_time ){

            finken_oscillating_last_time = stage_time;
            
            if( search_neighbor ){
            
                if ( go_down ){
                    if ( finken_system_set_point.distance_z > height_oscillating_down ){
                        finken_system_set_point.distance_z -= height_changing_rate;
                    } else {
                        finken_system_set_point.distance_z = height_oscillating_down;
                        go_down = false;
                    }
                
                } else {
                    if ( finken_system_set_point.distance_z < height_oscillating_up ){
                        finken_system_set_point.distance_z += height_changing_rate;
                    } else {
                        finken_system_set_point.distance_z = height_oscillating_up;
                        go_down = true;
                    }
                }
            }
        }
    }
}

//void update_finken_oscillating_model(void)
//{
//	if(finken_sensor_model.distance_z < 2.5) {
//		finken_oscillating_model.distance_z     = finken_sensor_model.distance_z;
//	}
//	
//  finken_oscillating_model.velocity_theta = finken_sensor_model.velocity_theta;
//  finken_oscillating_model.velocity_x     = finken_sensor_model.velocity_x;
//  finken_oscillating_model.velocity_y     = finken_sensor_model.velocity_y;
//}
//
//void send_finken_oscillating_model_telemetry(struct transport_tx *trans, struct link_device* link)
//{
//  trans=trans;
//  link=link;
//  DOWNLINK_SEND_FINKEN_SYSTEM_MODEL(
//    DefaultChannel,
//    DefaultDevice,
//    &finken_oscillating_model.distance_z,
//    &finken_oscillating_model.velocity_theta,
//    &finken_oscillating_model.velocity_x,
//    &finken_oscillating_model.velocity_y,
//    &finken_actuators_set_point.alpha,
//    &finken_actuators_set_point.beta,
//    &finken_actuators_set_point.thrust
//  );
//}
