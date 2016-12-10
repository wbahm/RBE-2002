#include "DriveController.h"
#include "Pinout.h"
#include "Rates.h"
#include "Conversion.h"
#include "DrivePWM.h"

DriveController::DriveController() 
{
  leftEnc = new Encoder(LEFT_ENC_REV?LEFT_ENC_CHAN_B:LEFT_ENC_CHAN_A,LEFT_ENC_REV?LEFT_ENC_CHAN_A:LEFT_ENC_CHAN_B);
  rightEnc = new Encoder(RIGHT_ENC_REV?RIGHT_ENC_CHAN_B:RIGHT_ENC_CHAN_A,RIGHT_ENC_REV?RIGHT_ENC_CHAN_A:RIGHT_ENC_CHAN_B);
  leftPID = new PID(100,0,0);
  rightPID = new PID(100,0,0);
}

long DriveController::getLeftEncCount() {
  return leftEnc->read();
}

long DriveController::getRightEncCount() {
  return rightEnc->read();
}

void DriveController::update() { //called each time in loop
  static unsigned long long lastLoopTime = 0;
  static long lastLeftPwr = 0;
  static long lastRightPwr = 0;
  static long lastLeftEnc = getLeftEncCount();
  static long lastRightEnc = getLeftEncCount();
  if((micros()-lastLoopTime)>MOTOR_CONTROLLER_PERIOD_US)
  {  
    //Get time diff and enc diff
    long long diffTime = micros()-lastLoopTime;
    lastLoopTime = micros();
    long long leftDiffTicks = (leftEnc->read())-lastLeftEnc;
    lastLeftEnc = leftEnc->read();
    long long rightDiffTicks = (rightEnc->read())-lastRightEnc;
    lastRightEnc = rightEnc->read();
    //compute velocity
    long long leftVel = ((leftDiffTicks*US_IN_SEC*UM_PER_TICK)/diffTime); //in mm/sec
    long long rightVel = ((rightDiffTicks*US_IN_SEC*UM_PER_TICK)/diffTime); //in mm/sec
    //compute PID
    long long leftpidOut = leftPID -> compute(leftSetpoint,leftVel);
    long long rightpidOut = rightPID -> compute(rightSetpoint,rightVel);
    //set output
    lastLeftPwr += leftpidOut;
    lastLeftPwr = constrain(lastLeftPwr,-65535,65535);
    lastRightPwr += rightpidOut;
    lastRightPwr = constrain(lastRightPwr,-65535,65535);
    if(leftSetpoint == 0)
      lastLeftPwr = 0;
    if(rightSetpoint == 0)
      lastRightPwr = 0;
    if(lastLeftPwr > 0)
    {
      setDrivePWM((unsigned int)lastLeftPwr, LEFT,FORWARD);
    }
    else
    {
      setDrivePWM((unsigned int)(-1*lastLeftPwr), LEFT,BACKWARD);
    }
    if(lastRightPwr > 0)
    {
      setDrivePWM((unsigned int)lastRightPwr, RIGHT,FORWARD);
    }
    else
    {
      setDrivePWM((unsigned int)(-1*lastRightPwr), RIGHT,BACKWARD);
    }
  }
}

void DriveController::setup() { //called during setup
  initDrivePWM();//sets up timer for PWM generation
}

void DriveController::setLeftSetpoint(long newLeft) {
  leftSetpoint = newLeft;
}

void DriveController::setRightSetpoint(long newRight) {
  rightSetpoint = newRight;
}

void DriveController::setBothSetpoints(long newLeft,long newRight) {
  setLeftSetpoint(newLeft);
  setRightSetpoint(newRight);
}

void DriveController::addToLeftSetpoint(long addLeft) {
  leftSetpoint += addLeft;
}

void DriveController::addToRightSetpoint(long addRight) {
  rightSetpoint += addRight;
}

void DriveController::addToBothSetpoints(long addLeft,long addRight) {
  addToLeftSetpoint(addLeft);
  addToRightSetpoint(addRight);
}

void DriveController::stop() {
  setBothSetpoints(0,0);
}
