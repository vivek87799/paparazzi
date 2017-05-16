#include "test_mod1.h"
#include "state.h"

//static struct FloatEulers* fe_heading; 

void heading_init(){
    fe_heading = 0;
}

void heading_periodic(){
    fe_heading = stateGetNedToBodyEulers_f();
}
