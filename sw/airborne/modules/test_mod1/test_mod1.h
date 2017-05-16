//#pragma once
#ifndef MOD1_H
#define MOD1_H

#include <std.h>

void heading_init(void);
void heading_periodic(void);
void send_heading_telemetry(struct transport_tx *trans, struct link_device* link);

static struct FloatEulers* fe_heading; 

#endif

