#pragma once

#include <modules/sonar/sonar_array_i2c.h>

struct sonar_diff_s {
	int16_t x, y;
};

extern struct sonar_values_s sonar_filtered_values;
extern struct sonar_diff_s sonar_filtered_diff_values;

void finken_sonar_filter_init(void);
void finken_sonar_filter_periodic(void);
