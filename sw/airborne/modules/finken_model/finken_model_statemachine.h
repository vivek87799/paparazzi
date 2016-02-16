#ifndef FINKEN_MODEL_STATEMACHINE_H
#define FINKEN_MODEL_STATEMACHINE_H

#include "std.h"
#include "modules/finken_model/finken_model_actuators.h"
#include "modules/finken_model/finken_model_system.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/finken_model/finken_model_oscillating.h"

#define FINKEN_SONAR_LOWER_BOUND 30
#define FINKEN_SONAR_UPPER_BOUND 60

extern bool finken_oscillating_mode;
extern uint8_t finite_state;
extern bool search_neighbor;
extern bool check_direction;

extern void finken_statemachine_model_init(void);
extern void finken_statemachine_model_periodic(void);

#endif
