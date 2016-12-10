#include <digitalIOPerformance.h>
#include "DrivePWM.h"
#include "WallMath.h"
#include "PID.h"

#define DEBUG_PRINT //used to enable debug print statements
#include "debug.h"

#include "Encoder.h"
#include "Rates.h"
#include "RobotOdometry.h"
#include "DriveController.h"

static const long long WALL_OFFSET_SETPOINT = (long)10*25.4;

PID* wallOffsetPid;
PID* wallThetaPid;

PID* turnPid;
DriveController* myDriveControl;

typedef enum RobotState {
  WALL_FOLLOW,
  TURN,
  STANDBY,
  SPEC_CASE, //Detected that front wall sensor maxed and other isn't
  FORWARD_DIST
};

RobotState currentState = STANDBY;
RobotState previousState = STANDBY;

void setup() {
  DebugBegin();
  DebugPrintln("Serial started");
  myDriveControl = DriveController::getInstance();
  myDriveControl->setup();
  wallInit();
  wallOffsetPid = new PID(3000000,0,0);//600000
  wallThetaPid = new PID(600000,0,0); 
  turnPid = new PID(1250000,0,0);
}
unsigned long long lastWallPID = 0;
//long long lastLeftSetpoint = SETPOINT;//+50000;
//long long lastRightSetpoint = SETPOINT;//-50000;
long long walloffsetpidOut = 0;
long long wallthetapidOut = 0;
void handleWallPID();
void handleTurn();
void handleSpecCase();
void handleForward();
float targetTurnHeading=0; //Rad
long long forwardTarget = 0;//mm dist

void loop() {
  switch(currentState)
  {
    case STANDBY://add btn later
    delay(1000);
    currentState = WALL_FOLLOW;
    //DebugPrintln("Standby");
    break;
    case SPEC_CASE:
    handleSpecCase();
    //DebugPrintln("Special");
    break;
    case WALL_FOLLOW:
    handleWallPID();
    //DebugPrintln("WallFollow");
    break;
    case TURN:
    //DebugPrintln("Turn");
    handleTurn();
    break;
    case FORWARD_DIST:
    handleForward();
    //DebugPrintln("ForwardDist");
    break;
    default:
    //DebugPrintln("Default");
    break;
  }
  myDriveControl->update();
  computeOdometry(myDriveControl);
/*  DebugPrint(getXLoc()); //mm
  DebugPrint('\t');
  DebugPrint(getYLoc()); //mm
  DebugPrint('\t');
  DebugPrintln(getTheta()*(180/PI)); //deg*/
}

void handleWallPID() {
  static long long stoppedTime = 0;
  static bool startTurn = false;
  if((micros()-lastWallPID) >WALL_FOLLOW_PERIOD_US)
  {
    lastWallPID = micros();
    WallState newState = getWallState(RIGHT_WALL);
    //DebugPrintln((newState.frontDist < (25.4*10))&&(newState.frontDist>5));
    if((newState.frontDist < (25.4*10))&&(newState.frontDist>5))
    {
      myDriveControl->stop(); //stop if wall in front is too close
      if(startTurn == false)
      {
        startTurn = true;
        stoppedTime = micros();
      }
      targetTurnHeading = getTheta()+(PI/2.0);
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
      
      /*DebugPrint(newState.wallDist/25.4);
      DebugPrint('\t');
      DebugPrint((newState.theta/PI)*0.180);
      DebugPrint('\t');
      DebugPrint('\t');
      DebugPrint((long)walloffsetpidOut);
      DebugPrint('\t');
      DebugPrint((long)wallthetapidOut);
      DebugPrint('\t');
      DebugPrint('\t');
      DebugPrint((long)lastLeftSetpoint);
      DebugPrint('\t');
      DebugPrint((long)lastRightSetpoint);
      DebugPrint('\t');
      DebugPrint('\t');*/

    }
  }
}

static const float TURN_TOL = 1*(PI/180.0); //Rad

void handleTurn() {
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

void handleSpecCase() {
  static enum SpecCaseStep{
    FORWARD_FIRST,
    TURN_FIRST,
    FORWARD_SECOND,
    TURN_SECOND,
    FORWARD_THIRD,
    DONE
  } myState = FORWARD_FIRST;
  switch(myState)
  {
    case FORWARD_FIRST:
    previousState = currentState;
    //set forward target
    forwardTarget = (25.4*10);
    currentState = FORWARD_DIST;
    myState = TURN_FIRST;
    break;
    case TURN_FIRST:
    previousState = currentState;
    //set turn target
    targetTurnHeading = getTheta()-(PI/2);
    currentState = TURN;
    myState = FORWARD_SECOND;
    break;
    case FORWARD_SECOND:
    previousState = currentState;
    //set forward target
    forwardTarget = (25.4*22);
    currentState = FORWARD_DIST;
    myState = TURN_SECOND;
    break;
    case TURN_SECOND:
    previousState = currentState;
    //set turn target
    targetTurnHeading = getTheta()-(PI/2);
    currentState = TURN;
    myState = FORWARD_THIRD;
    break;
    case FORWARD_THIRD:
    previousState = currentState;
    //set forward target
    forwardTarget = (25.4*15);
    currentState = FORWARD_DIST;
    myState = DONE;
    break;
    case DONE:
    previousState = currentState;
    currentState = WALL_FOLLOW;
    myState = FORWARD_FIRST;
    break;
    default:
    break;
  }
}

static const int FORWARD_TOL = 0.5*25.4;//mm
void handleForward() {
  static bool started = false;
  static long startX = 0; //mm
  static long startY = 0; //mm
  static long long stopTime = 0;
  static bool stopping = false;
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
