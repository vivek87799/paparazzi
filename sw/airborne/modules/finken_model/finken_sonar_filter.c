#include <modules/finken_model/finken_sonar_filter.h>
#include <modules/sonar/sonar_array_i2c.h>

#ifndef BACKLOG
#define BACKLOG 2
#endif
#ifndef MIN_ORDER
#define MIN_ORDER BACKLOG + 0
#endif

enum Axes {X, Y};

uint16_t sonar_filtered_values[BACKLOG][SONAR_END];
uint32_t virtSonars[4];
int16_t sonar_filtered_diff_values[2];

static int lpI, lpA;
static uint16_t lowPassBuf[SONAR_END][FINKEN_SONAR_LOW_PASS_SIZE];
static int16_t lowPassDiff[2][FINKEN_SONAR_DIFF_LOW_PASS_SIZE];

static void setSonarFilterValue(enum Sonars sonar, uint16_t value) {
  for(int i=1; i<BACKLOG;i++)
	  sonar_filtered_values[i][sonar] = sonar_filtered_values[i-1][sonar];
	sonar_filtered_values[0][sonar] = value;
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

static uint16_t robustMinFilter(enum Sonars sonar) {
  uint16_t min[MIN_ORDER];
  uint8_t sonarOffset[] = {0, 3, 4};
  memset(min, 0xff, sizeof(min));
  for(unsigned int i=0;i<BACKLOG;i++)
    for(unsigned int s=0;s<sizeof(sonarOffset);s++)
      for(unsigned int j=0;j<MIN_ORDER;j++)
        if(sonar_filtered_values[i][sonar+sonarOffset[s]] <= min[j]){
          for(unsigned int k=j+1;k<MIN_ORDER;k++)
            min[k] = min[k-1];
          min[j] = sonar_filtered_values[i][sonar+sonarOffset[s]];
          break;
        }
  return min[MIN_ORDER-1];
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
  memset(sonar_filtered_values, 0xff, sizeof(sonar_filtered_values));
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
	if(SONAR_END>SONAR_LEFT)
    for(int i = 0; i<= SONAR_LEFT;i++)
		  virtSonars[i] = robustMinFilter(i);
	else {
		memcpy(virtSonars, sonar_filtered_values[0], sizeof(virtSonars));
	}
	int16_t x = diffRangeFilter(virtSonars[SONAR_BACK], virtSonars[SONAR_FRONT]);
	int16_t y = diffRangeFilter(virtSonars[SONAR_RIGHT], virtSonars[SONAR_LEFT]);
	sonar_filtered_diff_values[X]= diffLowPassFilter(X, x);
	sonar_filtered_diff_values[Y]= diffLowPassFilter(Y, y);
}
