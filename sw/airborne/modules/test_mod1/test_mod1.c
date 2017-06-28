#include "test_mod1.h"

//static struct FloatEulers* fe_heading; 

void heading_init(void){
    fe_heading = 0;
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_HEADING, send_heading_telemetry);
}

void heading_periodic(void){
    fe_heading = stateGetNedToBodyEulers_f();
}

void send_heading_telemetry(struct transport_tx *trans, struct link_device* link){
    trans=trans;
    link=link;

    if(fe_heading != 0 ){
    	float heading = fe_heading->psi;

    	DOWNLINK_SEND_HEADING(
    	DefaultChannel,
    	DefaultDevice,
		&heading
	);
    }

}
