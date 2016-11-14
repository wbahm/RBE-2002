#ifndef WALL_MATH_H
#define WALL_MATH_H

#include <Arduino.h>

//(a/(1000*(x+b)))-42 //return dist in mm
#define IR_ZERO_DIST(x) ((48955435/(x+33))-4200)/1000 //return dist in mm
#define IR_FOUR_DIST(x) ((44177767/(x-66))-4200)/1000 //return dist in mm

static void wallInit() {
  analogReference(INTERNAL2V56);
   ADCSRA &= ~0x7;  // remove clk prescaler arduino set
   ADCSRA |= (1<<ADPS2);//sets prescaler to 16; using full 1 Mhz rate recommended
}

static long Y_SENS = (long)(25.4*6); //y Offset between sensors
static long X_SENS = (long)(25.4*0); //x offset from sensor to center of robot

static long calculate_x_offset(long d1, long d2) {
  return ((Y_SENS/sqrt((Y_SENS*Y_SENS)+((d2-d1)*(d2-d1))))*(((d2+d1)/2)+X_SENS));
}

static long getXOffset() {// Add averaging if needed
  long d1 = IR_ZERO_DIST(analogRead(A0));
  long d2 = IR_FOUR_DIST(analogRead(A4));
  return calculate_x_offset(d1,d2);
}


#endif
