#ifndef WALL_MATH_H
#define WALL_MATH_H

#include <Arduino.h>

//(a/(1000*(x+b)))-42 //return dist in mm
#define IR_ZERO_DIST(x) (((48955435/(x+33))-4200)/1000) //return dist in mm
#define IR_ONE_DIST(x) (((49281348/(x+9))-4200)/1000) //return dist in mm
#define IR_TWO_DIST(x) (((50133745/(x+20))-4200)/1000) //return dist in mm
#define IR_THREE_DIST(x) ((x>70)?(((81641309/(x-65))-4200)/1000):(60*25)) //return dist in mm
#define IR_FOUR_DIST(x) ((x>70)?(((44177767/(x-66))-4200)/1000):(60*25)) //return dist in mm

static void wallInit() {
  analogReference(INTERNAL2V56); //Better range for IR sensors which max out at 2.25v
   ADCSRA &= ~0x7;  // remove clk prescaler arduino set
   ADCSRA |= (1<<ADPS2);//sets prescaler to 16; using full 1 Mhz rate recommended
}

static long Y_SENS = (long)(25.4*6); //y Offset between sensors
static long X_SENS = (long)(25.4*4.25); //x offset from sensor to center of robot

static long calculateXOffset(long d1, long d2, long k1) {
  if(k1<Y_SENS)
  return ((Y_SENS/k1)*(((d2+d1)/2)+X_SENS));
  else
  return ((d2+d1)/2)+X_SENS;
}
static long Y_SENS_2 = (long)(25.4*4.25); //y offset to front sensor; mm
static long calculateFrontDist(long d3, long k1) {
  return (2*Y_SENS_2/k1)*(d3+Y_SENS_2);
}

typedef enum WallSide {
  LEFT_WALL,
  RIGHT_WALL
};

typedef struct WallState {
  long frontDist; //mm
  long wallDist; //mm
  long theta; //mRad
  bool specCase; //true if special case(front inf. and back not
};

static const long MAX_IR_DIST = 14*25.4; //mm
static WallState getWallState(WallSide side) {// Add averaging if needed
  static long d1;
  static long d2;
  static long d3;
  if(side == LEFT_WALL)
  {
    d1 = IR_ZERO_DIST(analogRead(A0));
    d2 = IR_FOUR_DIST(analogRead(A4));
  }
  else
  {
    d1 = IR_ONE_DIST(analogRead(A1));
    d2 = IR_TWO_DIST(analogRead(A2));
  }
  d3 = IR_THREE_DIST(analogRead(A3)); //front sensor
  WallState newState;
  if((d1>MAX_IR_DIST)&&(d2<MAX_IR_DIST))
  {
    newState.specCase = true;
  }
  else
  {
    newState.specCase = false;
  }
  
  long k1 = sqrt((Y_SENS*Y_SENS)+((d2-d1)*(d2-d1)));
  newState.wallDist = calculateXOffset(d1,d2,k1);
  newState.theta = 1000*((PI/2)-acos((d2-d1)/(float)Y_SENS));
  if(d3 > MAX_IR_DIST)
  {
    newState.frontDist = -1;
    return newState;
  }
  else
  {
    newState.frontDist = calculateFrontDist(d3, k1); 
  }  
  return newState;
}


#endif
