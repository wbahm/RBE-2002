#include "TurnState.h"
#include "RobotOdometry.h"
#include "GlobalInstances.h"
TurnState::TurnState() {
  targetTurnHeading=0;
  turnPid = new PID(1250000,0,0);
}

void TurnState::handle() {
  static long long stoppedTime = 0;
  static bool doneTurn = false;
  //Serial.println("Turn");
  if(targetTurnHeading>(2*PI))
  {
    targetTurnHeading -= (2*PI);
  }
  float currentHeading = getTheta();
  float absError = abs(currentHeading-targetTurnHeading);
  if(absError>PI) absError = (2*PI)-absError;
  long long pidOut = abs(turnPid -> compute(0,abs(currentHeading)*10000)); //scaled because pid uses long longs
  if((targetTurnHeading>currentHeading) xor (abs(targetTurnHeading-currentHeading)>PI))
  {
    //turn counter clockwise
    myDriveControl->setLeftSetpoint(-1*pidOut);
    myDriveControl->setRightSetpoint(pidOut);
  }
  else
  {
    //turn clockwise
    myDriveControl->setLeftSetpoint(pidOut);
    myDriveControl->setRightSetpoint(-1*pidOut);
  }
  if(abs(getTheta()-targetTurnHeading)<TURN_TOL)
  {
    if(doneTurn == false)
    {
      doneTurn = true;
      stoppedTime = micros();
    }
      myDriveControl->stop(); //stop motors
      if((micros()-stoppedTime)>500000)
      {
        currentState = previousState;
        previousState = TURN;
        doneTurn = false;
      }
  }
}

void TurnState::setTurnHeading(float newHeading) {
  targetTurnHeading = newHeading;
}
