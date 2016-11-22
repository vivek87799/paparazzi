#include <modules/ws2801/ws2801.h>

#include <led.h>
#include <string.h>

#ifndef LEDS
	#warning Undefined number of leds, Please set WS2801_LEDS to the amount of LEDS in your chain!
#endif
enum RGB {
	RGB_START = 0,
	B=0,
	G,
	R,
	RGB_END
};

enum State {
	clockIdle,
	clockStart,
	clockHigh,
	clockLow
};

enum Logic {
	low = 0,
	high
};

#ifdef COLOR_FORMAT_BGR
  #define COLOR_INC -1
  #define COLOR_START RGB_END-1
  #define COLOR_END RGB_START-1
#else
  #define COLOR_INC 1
  #define COLOR_START RGB_START
  #define COLOR_END RGB_END
	
#endif

#define START LEDS - 4


#ifndef TRACK_COLORS
	static uint8_t trackColors[4][RGB_END] = {{0, 255, 0}, {0, 255, 0}, {255, 0, 0}, {255, 0, 0} };	
#else
	static uint8_t trackColors[4][RGB_END] = {TRACK_COLORS};	
#endif

#ifdef ID_COLOR
  #if LEDS > 4
    static uint8_t acColor[RGB_END] = {ID_COLOR};
  #endif
#endif
//static uint8_t white[WS2801_LEDS][RGB_END]   = {{255, 255, 255}, {255, 255, 255}, {255, 255, 255}, {0, 0, 0} };

static uint8_t colorBuffer[LEDS][RGB_END];
static int8_t currLed, currColor, currBit, clockState, startTime;

uint8_t ws2801_ledNr;
uint8_t ws2801_red;
uint8_t ws2801_green;
uint8_t ws2801_blue;

static void setClock(enum Logic value) {
	if(value)
		LED_OFF(6);
	else
		LED_ON(6);
}

static void setData(enum Logic value) {
	if(value)
		LED_OFF(7);
	else
		LED_ON(7);
}

static void start(void) {
	clockState = clockStart;
	startTime = 150;
}

void ws2801_init(void) {
	currLed = 0;
	currColor = 0;
	currBit = 8;
#ifdef ID_COLOR
  #if LEDS > 4
    memcpy(colorBuffer[0], acColor, sizeof(acColor));
  #endif
#endif
	clockState = clockIdle;
	ws2801_deactivateLeds();
}

void ws2801_event(void) {
	switch(clockState) {
		case(clockStart):
			setClock(low);
			if(startTime--==0) {
				setData(colorBuffer[currLed][currColor] & (1 << --currBit) );
				clockState = clockHigh;
			}
			break;
		case(clockHigh):
			setClock(high);
			clockState = clockLow; 
			if( currBit == 0 ) {
				currColor+=COLOR_INC;
				currBit=8;
			}
				
			if(currColor==COLOR_END) {
				currLed++;
				currColor = COLOR_START;
			}
			if(currLed == LEDS) {
				currLed = 0;
				clockState = clockIdle;
			}

			break;
		case(clockLow): 
			setClock(low);
			setData(colorBuffer[currLed][currColor] & (1 << --currBit) );
			clockState = clockHigh;
			break;
		default: 
			setClock(high);
			setData(low);
			break;
	}

}

enum Errors ws2801_setColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
	if(led >= LEDS)
		return NO_SUCH_LED;
	if(clockState != clockIdle)
		return BUSY;
	colorBuffer[led+START][R] = r;
	colorBuffer[led+START][G] = g;
	colorBuffer[led+START][B] = b;
	start();
	return SUCCESS;
}

int ws2801_activateLeds(void){
	memcpy(colorBuffer+START, trackColors, sizeof(trackColors));
	start();
	return 0;
}

int ws2801_deactivateLeds(void){
	memset(colorBuffer+START, 0, sizeof(trackColors));
	start();
	return 0;
}


void ws2801_update(uint8_t ledNr) {
	ws2801_ledNr=ledNr;
	ws2801_setColor(ledNr, ws2801_red, ws2801_green, ws2801_blue);
}
