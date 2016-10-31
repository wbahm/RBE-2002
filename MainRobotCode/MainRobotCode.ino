#include "DrivePWM.h"
void setup() {
  initDrivePWM(LEFT);
  /*pinMode(3, OUTPUT);
  pinMode(11, OUTPUT);
  TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(WGM22) | _BV(CS22);
  OCR2A = 180;
  OCR2B = 50;*/
}

void loop() {
  //for(int i = 0; i<30000;i++)
  //{
    setDrivePWM(65535,LEFT);
    delay(500);
    setDrivePWM(0,LEFT);
    delay(500);
  //}
//  setDrivePWM(30000,LEFT);
 // delay(100);
}
