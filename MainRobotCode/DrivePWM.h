#ifndef DRIVE_PWM
#define DRIVE_PWM
#include <Arduino.h>
typedef enum DriveSide {
  LEFT,
  RIGHT
};

#define PWM_FREQ 16000
#define CLOCK_FREQ 16000000
#define PWM_COUNT (((CLOCK_FREQ/PWM_FREQ))-1)

static void initDrivePWM(DriveSide side) {
  //COM3A1 = 1
  //COM3A0 = 1
  //WGM33 = 1
  //WGM32 = 1
  //WGM31 = 1
  //WGM30 = 0
  
 if(side == LEFT)
 {
    pinMode(5,OUTPUT);
    TCCR3B = _BV(WGM33)|_BV(WGM32)|_BV(CS30);
    TCCR3A = _BV(COM3A1)|_BV(WGM31);//WGM3_14; //COM3 and WGM3 = 14 _BV(COM3A0)
    ICR3 = PWM_COUNT; // to count up to
    OCR3A = PWM_COUNT/2; //default to 50% duty cycle
 }
 else if (side == RIGHT)
 {
    
 }
}

static void setDrivePWM(unsigned int duty_cycle, DriveSide side) { 
  //Sets the new PWM duty cycle; duty cycle scaled from 0 to 65535 for 0 - 100 percent
  if(side == LEFT)
  {
    TCNT3 = 0;
    OCR3A = ((duty_cycle*PWM_COUNT)/65535);
  }
  else if (side == RIGHT)
  {
    
  }
}

#endif
