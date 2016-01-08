#pragma once

#include <modules/sonar/sonar_array_i2c.h>

extern struct sonar_values_s sonar_filtered_values;

void finken_sonar_filter_init(void);
void finken_sonar_filter_periodic(void);
