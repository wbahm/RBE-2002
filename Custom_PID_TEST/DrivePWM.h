#ifndef DRIVE_PWM_H
#define DRIVE_PWM_H

#include <Arduino.h>
#include <digitalIOPerformance.h>

typedef enum DriveSide {
  LEFT = 5, //Pin numbers
  RIGHT = 2
};

typedef enum DriveDirection {
  FORWARD = 1,
  BACKWARD = 0
};

#define LEFT_DIR_PIN 4
#define LEFT_PWM_PIN 2 //can't change because implemented at timer register lvl
#define RIGHT_DIR_PIN 6
#define RIGHT_PWM_PIN 5 //can't change because implemented at timer register lvl

#define LEFT_REVERSED 0
#define RIGHT_REVERSED 0

//PWM defines
#define PWM_FREQ 16000
#define CLOCK_FREQ 16000000
static const unsigned int PWM_COUNT = (((CLOCK_FREQ/PWM_FREQ))-1);// make constant to avoid recomputing each time

static void initDrivePWM() {
    pinMode(LEFT_PWM_PIN,OUTPUT);
    pinMode(RIGHT_PWM_PIN,OUTPUT);
    pinModeFast(LEFT_DIR_PIN,OUTPUT);
    pinModeFast(RIGHT_DIR_PIN,OUTPUT);
    TCCR3B = _BV(WGM33)|_BV(WGM32)|_BV(CS30);
    TCCR3A = _BV(COM3A1)|_BV(COM3B1)|_BV(WGM31);//WGM3_14; //COM3 and WGM3 = 14 _BV(COM3A0)
    ICR3 = PWM_COUNT; // to count up to
    OCR3A = 0; //default to off
    OCR3B = 0; //default to off
    
}

static void setDrivePWM(unsigned int duty_cycle, DriveSide side,DriveDirection newDir) { 
  //Sets the new PWM duty cycle; duty cycle scaled from 0 to 65535 for 0 - 100 percent
  constrain(duty_cycle,0,65535);
  if(side == LEFT)
  {
    OCR3A = ((duty_cycle*(unsigned long)PWM_COUNT)/65535);
    if(newDir == FORWARD)
    {
      if(LEFT_REVERSED)
        digitalWriteFast(LEFT_DIR_PIN,BACKWARD);
      else
        digitalWriteFast(LEFT_DIR_PIN,FORWARD);
    }
    else if(newDir == BACKWARD)
    {
      if(LEFT_REVERSED)
        digitalWriteFast(LEFT_DIR_PIN,FORWARD);
      else
        digitalWriteFast(LEFT_DIR_PIN,BACKWARD);
    }
  }
  else if (side == RIGHT)
  {
    OCR3B = ((duty_cycle*(unsigned long)PWM_COUNT)/65535);
    if(newDir == FORWARD)
    {
      if(RIGHT_REVERSED)
        digitalWriteFast(RIGHT_DIR_PIN,BACKWARD);
      else
        digitalWriteFast(RIGHT_DIR_PIN,FORWARD);
    }
    else if(newDir == BACKWARD)
    {
      if(RIGHT_REVERSED)
        digitalWriteFast(RIGHT_DIR_PIN,FORWARD);
      else
        digitalWriteFast(RIGHT_DIR_PIN,BACKWARD);
    }
  }
}

static void stopDrive() {
  setDrivePWM(0, LEFT,FORWARD);
  setDrivePWM(0, RIGHT,FORWARD);
}

#endif

