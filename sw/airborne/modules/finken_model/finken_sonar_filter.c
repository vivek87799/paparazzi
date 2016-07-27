#include <modules/finken_model/finken_sonar_filter.h>
#include <modules/sonar/sonar_array_i2c.h>

enum Axes {X, Y};

uint16_t sonar_filtered_values[SONAR_END];
int16_t sonar_filtered_diff_values[2];

static int lpI, lpA;
static uint16_t lowPassBuf[SONAR_END][FINKEN_SONAR_LOW_PASS_SIZE];
static int16_t lowPassDiff[2][FINKEN_SONAR_DIFF_LOW_PASS_SIZE];

static void setSonarFilterValue(enum Sonars sonar, uint16_t value) {
	sonar_filtered_values[sonar] = value;
	/*switch (sonar) {
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
	}*/
}

static uint16_t getSonarValue(enum Sonars sonar) {
	return sonar_values[sonar];
	/*switch (sonar) {
	case (SONAR_FRONT): return sonar_values.front;
	case (SONAR_RIGHT): return sonar_values.right;
	case (SONAR_BACK ): return sonar_values.back;
	case (SONAR_LEFT ): return sonar_values.left;
	default:            return FINKEN_SONAR_MAX_DIST;
	}*/
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

static int16_t diffLowPassFilter(enum Axes axis, int16_t value) {
	lowPassDiff[axis][lpA++] = value;
	lpA%=FINKEN_SONAR_DIFF_LOW_PASS_SIZE;
	int16_t sum = 0;
	for(unsigned int i=0;  i<FINKEN_SONAR_DIFF_LOW_PASS_SIZE; i++)
		sum +=lowPassDiff[axis][i];
	sum/=FINKEN_SONAR_DIFF_LOW_PASS_SIZE;
	return sum;
}

static int16_t diffRangeFilter(uint16_t a, uint16_t b) {
	if(a>FINKEN_SONAR_DIFF_GOAL_DIST)
		a = FINKEN_SONAR_DIFF_GOAL_DIST;
	if(b>FINKEN_SONAR_DIFF_GOAL_DIST)
		b = FINKEN_SONAR_DIFF_GOAL_DIST;
	int16_t value = (int16_t)a - b;
	return value;
}


void finken_sonar_filter_init(void) {
	lpI = 0;
	lpA = 0;
	for(unsigned int sonar = SONAR_START; sonar < SONAR_END; sonar++)
		for(unsigned int i = 0; i < FINKEN_SONAR_LOW_PASS_SIZE; i++)
			lowPassBuf[sonar][i] = FINKEN_SONAR_MAX_DIST;
	for(unsigned int axis = X; axis <= Y; axis++)
		for(unsigned int i = 0; i < FINKEN_SONAR_DIFF_LOW_PASS_SIZE; i++)
			lowPassDiff[axis][i] = 0;
}

void finken_sonar_filter_periodic(void) {
	for(unsigned int sonar = SONAR_START; sonar < SONAR_END; sonar++) {
		uint16_t value = getSonarValue(sonar);
		value = rangeFilter(value);
		value = lowPassFilter(sonar, value);
		setSonarFilterValue(sonar, value);
	}
	uint32_t virtFront = 2*sonar_filtered_values[SONAR_FRONT] + sonar_filtered_values[SONAR_FRONT_LEFT] + sonar_filtered_values[SONAR_FRONT_RIGHT];
	virtFront /=4;
	uint32_t virtBack  = 2*sonar_filtered_values[SONAR_BACK] + sonar_filtered_values[SONAR_BACK_LEFT] + sonar_filtered_values[SONAR_BACK_RIGHT];
	virtBack /=4;
	uint32_t virtLeft  = 2*sonar_filtered_values[SONAR_LEFT] + sonar_filtered_values[SONAR_FRONT_LEFT] + sonar_filtered_values[SONAR_BACK_LEFT];
	virtLeft /=4;
	uint32_t virtRight = 2*sonar_filtered_values[SONAR_RIGHT] + sonar_filtered_values[SONAR_FRONT_RIGHT] + sonar_filtered_values[SONAR_BACK_RIGHT];
	virtRight /=4;
	int16_t x = diffRangeFilter(virtBack, virtFront);
	int16_t y = diffRangeFilter(virtRight, virtLeft);
	sonar_filtered_diff_values[X]= diffLowPassFilter(X, x);
	sonar_filtered_diff_values[Y]= diffLowPassFilter(Y, y);
}
