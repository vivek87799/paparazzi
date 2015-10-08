#ifndef FINKEN_MODEL_OSCILLATING_H
#define FINKEN_MODEL_OSCILLATING_H

#include "std.h"
#include "modules/finken_model/finken_model_actuators.h"
#include "modules/finken_model/finken_model_system.h"
#include "mcu_periph/link_device.h"
#include "subsystems/datalink/transport.h"
#include "subsystems/datalink/telemetry.h"

extern bool finken_oscillating_mode;
extern uint16_t finken_oscillating_last_time;
extern bool search_neighbor;

extern void finken_oscillating_model_init(void);
extern void finken_oscillating_model_periodic(void);

//void update_finken_oscillating_model(void);
//extern void send_finken_oscillating_model_telemetry(struct transport_tx *trans, struct link_device* link);

#endif