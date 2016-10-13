#pragma once

#define FINKEN_SONAR_DIFF_FREE_DIST (FINKEN_SONAR_DIFF_GOAL_DIST*FINKEN_SONAR_DIFF_FREE_FACTOR)
#define FINKEN_SONAR_DIFF_GUARD_DIST (FINKEN_SONAR_DIFF_GOAL_DIST*FINKEN_SONAR_DIFF_GUARD_FACTOR)

#include <stdint.h>

extern uint32_t virtSonars[4];
extern int16_t sonar_filtered_diff_values[];

void finken_sonar_filter_init(void);
void finken_sonar_filter_periodic(void);
