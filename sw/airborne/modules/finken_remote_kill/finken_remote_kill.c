#include <modules/finken_remote_kill/finken_remote_kill.h>

#include <std.h>
#include <firmwares/rotorcraft/autopilot.h>
#include <subsystems/datalink/downlink.h>
#include <messages.h>
#include <dl_protocol.h>
#include <modules/ws2801/ws2801.h>


void finken_remote_kill_init( void ) {

}

void finken_remote_kill_action(void) {
	if(DL_KILL_ac_id(dl_buffer) != AC_ID)
		return;

	if(DL_KILL_kill_mode(dl_buffer))
		autopilot_set_mode(AP_MODE_KILL);
	else
		autopilot_set_mode(AP_MODE_NAV);
	ws2801_activateLeds();
}
