#include "ForwardState.h"
#include "GlobalInstances.h"
#include "RobotOdometry.h"

ForwardState::ForwardState() {
  started = false;
  startX = 0; //mm
  startY = 0; //mm
  stopTime = 0;
  stopping = false;
  forwardTarget = 0;//mm dist
}

void ForwardState::handle() {
  if(started == false)
  {
    startX = getXLoc();
    startY = getYLoc();
    started = true;
  }
  long diffX = abs(getXLoc()-startX);
  long diffY = abs(getYLoc()-startY);
  long distTravel = sqrt((diffX*diffX)+(diffY*diffY));
  if((forwardTarget-distTravel)>FORWARD_TOL)
  {
    myDriveControl->setBothSetpoints(DriveController::DEFAULT_SETPOINT,DriveController::DEFAULT_SETPOINT);
  }
  else if((distTravel-forwardTarget)>FORWARD_TOL)
  {
    myDriveControl->setBothSetpoints(-1*DriveController::DEFAULT_SETPOINT,-1*DriveController::DEFAULT_SETPOINT);
  }
  else
  {
    //in tol
    myDriveControl->stop();
    if(stopping == false)
    {
      stopping = true;
      stopTime = micros();
    }
    if((micros()-stopTime)>500000)
    {
      stopping = false;
      started = false;
      currentState = previousState;
      previousState = FORWARD_DIST;
    }
  }
}
void ForwardState::setForwardTarget(long long newTarget) {
  forwardTarget = newTarget;
}
