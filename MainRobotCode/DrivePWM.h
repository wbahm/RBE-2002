#ifndef DRIVE_PWM
#define DRIVE_PWM
#include <Arduino.h>
typedef enum DriveSide {
  LEFT = 5, //Pin number
  RIGHT = 2
};

#define PWM_FREQ 16000
#define CLOCK_FREQ 16000000
#define PWM_COUNT (((CLOCK_FREQ/PWM_FREQ))-1)

static void initDrivePWM() {
    pinMode(LEFT,OUTPUT);
    pinMode(RIGHT,OUTPUT);
    TCCR3B = _BV(WGM33)|_BV(WGM32)|_BV(CS30);
    TCCR3A = _BV(COM3A1)|_BV(COM3B1)|_BV(WGM31);//WGM3_14; //COM3 and WGM3 = 14 _BV(COM3A0)
    ICR3 = PWM_COUNT; // to count up to
    OCR3A = PWM_COUNT/2; //default to 50% duty cycle
    OCR3B = PWM_COUNT/2; //default to 50% duty cycle
}

static void setDrivePWM(unsigned int duty_cycle, DriveSide side) { 
  //Sets the new PWM duty cycle; duty cycle scaled from 0 to 65535 for 0 - 100 percent
  if(side == LEFT)
  {
    OCR3A = ((duty_cycle*PWM_COUNT)/65535);
  }
  else if (side == RIGHT)
  {
    OCR3B = ((duty_cycle*PWM_COUNT)/65535);
  }
}

#endif
