#include <modules/finken_wall_avoid/finken_wall_avoid.h>

#define T 1.0f/FINKEN_WALL_AVOID_UPDATE_FREQ
#define T1 FINKEN_WALL_AVOID_CONTROL_DELAY_TIME
#define Tv FINKEN_WALL_AVOID_CONTROL_HOLD_TIME
#define Kp FINKEN_WALL_AVOID_CONTROL_GAIN

#define a0 -0.1031288//(T-2.0f*T1)/(T+2.0f*T1)
#define b0 51.9420027//Kp*(T-2.0f*Tv)/(T+2.0f*T1)
#define b1 -50.675209//Kp*(T+2.0f*Tv)/(T+2.0f*T1)

static const float maxControlRoll  = 20.0f;
static const float maxControlPitch = 20.0f;
static const float guardDist       =  0.7f;
static const float goalDist        =  1.3f;
static const float freeDist        =  0.9f*1.3f;
static const float maxDist         =  1.5f;

float pitchError_k1, pitch_k1, rollError_k1, roll_k1;
float pitchInDamped, rollInDamped;

static float rollControl(float rollError) {
	float roll = -a0*roll_k1 + b1*rollError + b0*rollError_k1;
	roll = (roll < -maxControlPitch) ? -maxControlPitch : roll;
	roll = (roll > maxControlPitch) ? maxControlPitch : roll;
	roll_k1 = roll;
	rollError_k1 = rollError;
	return roll;
}

static float rollWallAvoid(float rollIn, float distXLeft, float distXRight) {
	distXLeft = distXLeft<maxDist?distXLeft:maxDist;
	distXRight  = distXRight<maxDist?distXRight:maxDist;
	float distXControlLeft = distXLeft>goalDist?freeDist:distXLeft;
	float distXControlRight  = distXRight>goalDist?freeDist:distXRight;
	float distXControlDiff  = 0.0f-(distXControlRight-distXControlLeft);
	float newRoll = rollControl(distXControlDiff);
	/*rollIn = (rollIn < -maxRCPitch) ? -maxRCPitch : rollIn;
	rollIn = (rollIn > maxRCPitch)  ?  maxRCPitch : rollIn;
	rollIn = (rollIn< deadRCPitch && rollIn > -deadRCPitch) ? 0.0f : rollIn;*/
  if (distXLeft < maxDist && rollIn > 0)
		rollIn*=(distXLeft-guardDist)/(maxDist-guardDist);
  if (distXRight < maxDist && rollIn < 0)
		rollIn*=(distXRight-guardDist)/(maxDist-guardDist);
	rollInDamped = rollIn;
	return newRoll+rollIn;
}

static float pitchControl(float pitchError) {
	float pitch = -a0*pitch_k1 + b1*pitchError + b0*pitchError_k1;
	pitch = (pitch < -maxControlPitch) ? -maxControlPitch : pitch;
	pitch = (pitch > maxControlPitch) ? maxControlPitch : pitch;
	pitch_k1 = pitch;
	pitchError_k1 = pitchError;
	return pitch;
}

static float pitchWallAvoid(float pitchIn, float distXFront, float distXBack) {
	distXFront = distXFront<maxDist?distXFront:maxDist;
	distXBack  = distXBack<maxDist?distXBack:maxDist;
	float distXControlFront = distXFront>goalDist?freeDist:distXFront;
	float distXControlBack  = distXBack>goalDist?freeDist:distXBack;
	float distXControlDiff  = 0.0f-(distXControlBack-distXControlFront);
	float newPitch = pitchControl(distXControlDiff);
	/*pitchIn = (pitchIn < -maxRCPitch) ? -maxRCPitch : pitchIn;
	pitchIn = (pitchIn > maxRCPitch)  ?  maxRCPitch : pitchIn;
	pitchIn = (pitchIn< deadRCPitch && pitchIn > -deadRCPitch) ? 0.0f : pitchIn;*/
  if (pitchIn > 0)
		pitchIn*=(distXFront-guardDist)/(maxDist-guardDist);
  if (pitchIn < 0)
		pitchIn*=(distXBack-guardDist)/(maxDist-guardDist);
	pitchInDamped = pitchIn;
	return newPitch+pitchIn;
}

void finken_wall_avoid_init() {
	pitchError_k1 = 0.0f;
	pitch_k1      = 0.0f;
	rollError_k1  = 0.0f;
	roll_k1       = 0.0f;
		
	register_periodic_telemetry(DefaultPeriodic, "FINKEN_WALL_AVOID", send_finken_wall_avoid_telemetry);
}

void finken_wall_avoid_periodic() {

	finken_actuators_set_point.pitch = pitchWallAvoid(finken_system_set_point.pitch, finken_sensor_model.distance_d_front/100.0, finken_sensor_model.distance_d_back/100.0);
	finken_actuators_set_point.roll = rollWallAvoid(finken_system_set_point.roll, finken_sensor_model.distance_d_left/100.0, finken_sensor_model.distance_d_right/100.0);

}


void send_finken_wall_avoid_telemetry(struct transport_tx *trans, struct link_device* link) {
	trans = trans;
	link = link;
	float front=finken_sensor_model.distance_d_front/100.0f;
	float back=finken_sensor_model.distance_d_back/100.0f;
	float left=finken_sensor_model.distance_d_left/100.0f;
	float right=finken_sensor_model.distance_d_right/100.0f;
	float z=POS_FLOAT_OF_BFP(finken_sensor_model.pos.z);
	DOWNLINK_SEND_FINKEN_WALL_AVOID(
	DefaultChannel,
	DefaultDevice,
	&z,
	&pitchInDamped,
	&rollInDamped,
	&pitch_k1,
	&roll_k1,
	&rollError_k1,
	&pitchError_k1,
	&front,
	&back,
	&left,
	&right,
	&finken_actuators_set_point.thrust
	);
}
