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
  unsigned long total = 0;
  for(int i = 0;i<4;i++)
  {
    total += (unsigned long) getXOffset();
  }
  total = total/4;
  startTime = micros()-startTime;
  float inch = total/25.4f;
  Serial.print(startTime);
  Serial.print('\t');
  Serial.println(inch);
}
