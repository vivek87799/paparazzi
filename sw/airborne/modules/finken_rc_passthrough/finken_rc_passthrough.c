#include <modules/finken_rc_passthrough/finken_rc_passthrough.h>

void finken_rc_passthrough_periodic() {
	finken_actuators_set_point.pitch = finken_system_set_point.pitch;
	finken_actuators_set_point.roll = finken_system_set_point.roll;
	finken_actuators_set_point.yaw = finken_system_set_point.yaw;
}
