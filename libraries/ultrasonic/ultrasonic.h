/*
  ultrasonic.h - Library for finding the distance to an object in inches.
*/

#ifndef ultrasonic_h
#define ultrasonic_h

#include "Arduino.h"

class ultrasonic {
public:
  ultrasonic(int pingPin, int echoPin);
  float distance();
private:
  int _pingPin;
  int _echoPin;
  float calc();
};

#endif

