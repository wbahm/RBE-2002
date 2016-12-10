#ifndef CONVERSION_H
#define CONVERSION_H
#include "Arduino.h"

/* This has the conversion factors for various units.
*/

#define US_IN_SEC 1000000 //uSec per sec
static const float WHEEL_DIAMETER_INCH = 2.74;
static const long UM_PER_IN = 25400;
static const float WHEEL_DIAMETER_UM = WHEEL_DIAMETER_INCH*UM_PER_IN;
static const float TICKS_PER_REV = 8245.81; //on output shaft
static const long UM_PER_TICK = (WHEEL_DIAMETER_UM*PI)/TICKS_PER_REV;

#endif
