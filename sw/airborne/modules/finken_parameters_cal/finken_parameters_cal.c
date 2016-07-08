#include <modules/finken_parameters_cal/finken_parameters_cal.h>

#include <std.h>
#include <firmwares/rotorcraft/autopilot.h>
#include <subsystems/datalink/downlink.h>
#include <messages.h>
#include <dl_protocol.h>
#include <modules/ws2801/ws2801.h>

float calParamsRoll;
float calParamsPitch;
float calParamsYaw;

void finken_parameters_cal_init( void ) {
	calParamsRoll = 0;
	calParamsPitch = 0;
	calParamsYaw = 0;	

}

void finken_parameters_cal_action(void) {
	if(DL_CALPARAMS_ac_id(dl_buffer) != AC_ID)
		return;

	calParamsRoll = DL_CALPARAMS_calParamsRoll(dl_buffer);
	calParamsPitch = DL_CALPARAMS_calParamsPitch(dl_buffer);
	calParamsYaw = DL_CALPARAMS_calParamsYaw(dl_buffer);
}
