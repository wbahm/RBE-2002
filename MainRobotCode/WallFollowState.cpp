#include "WallFollowState.h"
#include "WallMath.h"
#include "RobotOdometry.h"
#include "Rates.h"
#include "GlobalInstances.h"
#include "debug.h"

WallFollowState::WallFollowState() {
  stoppedTime  = 0;
  startTurn = false;
  lastLoopTime = 0;
  walloffsetpidOut = 0;
  wallthetapidOut = 0;
  wallOffsetPid = new PID(3000000,0,0);//600000
  wallThetaPid = new PID(600000,0,0); 
}

void WallFollowState::handle() {
  if((micros()-lastLoopTime) >WALL_FOLLOW_PERIOD_US)
  {
    lastLoopTime = micros();
    WallState newState = getWallState(RIGHT_WALL);
    //DebugPrintln((newState.frontDist < (25.4*10))&&(newState.frontDist>5));
    if(newState.frontDist < (25.4*6))
    {
      myDriveControl->stop(); //stop if wall in front is too close
      if(startTurn == false)
      {
        startTurn = true;
        stoppedTime = micros();
      }
      myTurnState->setTurnHeading(getTheta()+(PI/2.0));
      if((micros()-stoppedTime)>100000)
      {
        startTurn = false;
        currentState = TURN;
        previousState = WALL_FOLLOW;
        DebugPrintln("Back to turn");
      }
    }
    else if(newState.specCase)
    {
      currentState = SPEC_CASE;
    }
    else
    {
      walloffsetpidOut = wallOffsetPid -> compute(WALL_OFFSET_SETPOINT,newState.wallDist);
      wallthetapidOut = wallThetaPid -> compute(0,newState.theta);
      long newLeft = DriveController::DEFAULT_SETPOINT+wallthetapidOut-walloffsetpidOut;
      long newRight = DriveController::DEFAULT_SETPOINT+walloffsetpidOut-wallthetapidOut;
      myDriveControl->setLeftSetpoint(abs(newLeft));
      myDriveControl->setRightSetpoint(abs(newRight));
    }
  }
}
