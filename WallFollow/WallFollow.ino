#include <digitalIOPerformance.h>
#include "DrivePWM.h"
#include "WallMath.h"

#define DEBUG_PRINT
#include "debug.h"

void setup() {
  DebugBegin();
  DebugPrintln("Serial started");
  wallInit();
  initDrivePWM();//sets up timer for PWM generation
}
void loop() {
  long startTime = micros();
  WallState newState = getWallState(RIGHT_WALL);
  startTime = micros()-startTime;
  float inch = newState.wallDist/25.4f;
  Serial.print(startTime);
  Serial.print('\t');
  Serial.println(inch);
}
