#ifndef FINKEN_MODEL_OSCILLATING_H
#define FINKEN_MODEL_OSCILLATING_H

#include "std.h"
#include "modules/finken_model/finken_model_actuators.h"
#include "modules/finken_model/finken_model_system.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"
#include "modules/finken_model/finken_model_statemachine.h"

extern float height_oscillating_down;
extern float height_oscillating_up;
extern float middle;
extern bool go_down;
extern float height_changing_rate;

extern void finken_oscillating_model_init(void);
extern void finken_oscillating_model_periodic(void);

#endif
