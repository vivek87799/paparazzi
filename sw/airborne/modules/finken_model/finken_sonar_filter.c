#include <modules/finken_model/finken_sonar_filter.h>

struct sonar_values_s sonar_filtered_values;

static int lpI;
static uint16_t lowPassBuf[SONAR_END][FINKEN_SONAR_LOW_PASS_SIZE];

static void setSonarFilterValue(enum Sonars sonar, uint16_t value) {
	switch (sonar) {
	case (SONAR_FRONT):
		sonar_filtered_values.front = value;
		break;
	case (SONAR_RIGHT):
		sonar_filtered_values.right = value;
		break;
	case (SONAR_BACK):
		sonar_filtered_values.back = value;
		break;
	case (SONAR_LEFT):
		sonar_filtered_values.left = value;
		break;
	default:
		break;
	}
}

static uint16_t getSonarValue(enum Sonars sonar) {
	switch (sonar) {
	case (SONAR_FRONT): return sonar_values.front;
	case (SONAR_RIGHT): return sonar_values.right;
	case (SONAR_BACK ): return sonar_values.back;
	case (SONAR_LEFT ): return sonar_values.left;
	default:            return FINKEN_SONAR_MAX_DIST;
	}
}

static uint16_t rangeFilter(uint16_t value) {
	if(value<FINKEN_SONAR_MIN_DIST)
		return FINKEN_SONAR_MIN_DIST;
	if(value>FINKEN_SONAR_MAX_DIST)
		return FINKEN_SONAR_MAX_DIST;
	return value;
}

static uint16_t lowPassFilter(enum Sonars sonar, uint16_t value) {
	lowPassBuf[sonar][lpI++] = value;
	lpI%=FINKEN_SONAR_LOW_PASS_SIZE;
	uint16_t sum = 0;
	for(unsigned int i=0;  i<FINKEN_SONAR_LOW_PASS_SIZE; i++)
		sum +=lowPassBuf[sonar][i];
	sum/=FINKEN_SONAR_LOW_PASS_SIZE;
	return sum;
}

void finken_sonar_filter_init(void) {
	lpI = 0;
	for(unsigned int sonar = SONAR_START; sonar < SONAR_END; sonar++)
		for(unsigned int i = 0; i < FINKEN_SONAR_LOW_PASS_SIZE; i++)
			lowPassBuf[sonar][i] = 0;	
}

void finken_sonar_filter_periodic(void) {
	for(unsigned int sonar = SONAR_START; sonar < SONAR_END; sonar++) {
		uint16_t value = getSonarValue(sonar);
		value = rangeFilter(value);
		value = lowPassFilter(sonar, value);
		setSonarFilterValue(sonar, value);
	}
}
