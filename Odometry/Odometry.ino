#include <digitalIOPerformance.h>
#include "DrivePWM.h"
#include "WallMath.h"
#include "PID.h"

#define DEBUG_PRINT //used to enable debug print statements
#include "debug.h"

#include "Encoder.h"
Encoder leftEnc(20,19);
Encoder rightEnc(18,3);

#include "RobotOdometry.h"

unsigned long long lastContr = 0;
const unsigned long long contrFreq = 200; //hz
const unsigned long long contrT = 1000000/contrFreq;

static const long long UM_PER_TICK = 27;
static const long long US_TO_SEC = 1000000;
static const long long SETPOINT = 0.75*2.75*PI*2.54*10000; // um/sec -> 3.9 in/sec -> 0.46 rps
static const long long WALL_OFFSET_SETPOINT = (long)10*25.4;

PID* leftPid;
PID* rightPid;
long lastLeftPwr = 0;
long lastRightPwr = 0;
unsigned long long lastLeftEnc = 0;
unsigned long long lastRightEnc = 0;

PID* wallOffsetPid;
PID* wallThetaPid;

PID* turnPid;

typedef enum RobotState {
  WALL_FOLLOW,
  TURN,
  STANDBY,
  SPEC_CASE //Detected that front wall sensor maxed and other isn't
};

RobotState currentState = STANDBY;
RobotState previousState = STANDBY;

void setup() {
  DebugBegin();
  DebugPrintln("Serial started");
  wallInit();
  initDrivePWM();//sets up timer for PWM generation
  leftPid = new PID(100,0,0);
  rightPid = new PID(100,0,0);
  wallOffsetPid = new PID(3000000,0,0);//600000
  wallThetaPid = new PID(600000,0,0); 
  turnPid = new PID(600000,0,0);
}
unsigned long long lastWallPID = 0;
long long lastLeftSetpoint = SETPOINT;//+50000;
long long lastRightSetpoint = SETPOINT;//-50000;
long long walloffsetpidOut = 0;
long long wallthetapidOut = 0;
long long MAX_SETPOINT = 0.9*2.75*PI*2.54*10000;
long long MIN_SETPOINT = 0.3*2.75*PI*2.54*10000;
void handleWallPID();
void handleTurn();
float targetTurnHeading=0; //Rad
void loop() {
  switch(currentState)
  {
    case STANDBY://add btn later
    currentState = WALL_FOLLOW;
    break;
    case WALL_FOLLOW:
    handleWallPID();
    break;
    case TURN:
    handleTurn();
    break;
    default:
    1+1;
    break;
  }
  handleMotorControl(lastLeftSetpoint,lastRightSetpoint);
  computeOdometry(&leftEnc,&rightEnc);
/*  DebugPrint(getXLoc()); //mm
  DebugPrint('\t');
  DebugPrint(getYLoc()); //mm
  DebugPrint('\t');
  DebugPrintln(getTheta()*(180/PI)); //deg*/
}
void handleMotorControl(long long leftSetpoint,long long rightSetpoint) {
    if((micros()-lastContr)>contrT)
  {  
    //Get time diff and enc diff
    long long diffTime = micros()-lastContr;
    lastContr = micros();
    long long leftDiffTicks = leftEnc.read()-lastLeftEnc;
    lastLeftEnc = leftEnc.read();
    long long rightDiffTicks = rightEnc.read()-lastRightEnc;
    lastRightEnc = rightEnc.read();
    //compute velocity
    long long leftVel = ((leftDiffTicks*US_TO_SEC*UM_PER_TICK)/diffTime); //in mm/sec
    long long rightVel = ((rightDiffTicks*US_TO_SEC*UM_PER_TICK)/diffTime); //in mm/sec
    //compute PID
    long long leftpidOut = leftPid -> compute(leftSetpoint,leftVel);
    long long rightpidOut = rightPid -> compute(rightSetpoint,rightVel);
    //set output
    lastLeftPwr += leftpidOut;
    lastLeftPwr = constrain(lastLeftPwr,-65535,65535);
    lastRightPwr += rightpidOut;
    lastRightPwr = constrain(lastRightPwr,-65535,65535);
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

void handleWallPID() {
  if((micros()-lastWallPID) >(16667)) //60 hz
  {
    lastWallPID = micros();
    WallState newState = getWallState(RIGHT_WALL);
    DebugPrintln((newState.frontDist < (25.4*10))&&(newState.frontDist>5));
    if((newState.frontDist < (25.4*10))&&(newState.frontDist>5))
    {
      lastLeftSetpoint = 0; //stop if wall in front is too close
      lastRightSetpoint = 0;
      targetTurnHeading = getTheta()+(PI/2.0);
      currentState = TURN;
      previousState = WALL_FOLLOW;
      DebugPrintln("Back to turn");
    }
    else
    {
      walloffsetpidOut = wallOffsetPid -> compute(WALL_OFFSET_SETPOINT,newState.wallDist);
      wallthetapidOut = wallThetaPid -> compute(0,newState.theta);
      
      lastLeftSetpoint = (SETPOINT-walloffsetpidOut+wallthetapidOut);
      lastRightSetpoint = (SETPOINT+walloffsetpidOut-wallthetapidOut);
      lastLeftSetpoint = constrain(lastLeftSetpoint,MIN_SETPOINT,MAX_SETPOINT);
      lastRightSetpoint = constrain(lastRightSetpoint,MIN_SETPOINT,MAX_SETPOINT);
      
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
      static long lastLeftVal = leftEnc.read();
      long leftdiff = leftEnc.read()-lastLeftVal;
      lastLeftVal = leftEnc.read();
      static long lastRightVal = rightEnc.read();
      long rightdiff = rightEnc.read()-lastRightVal;
      lastRightVal = rightEnc.read();
      /*DebugPrint((leftdiff/8245.81)*(1000000/16667.0));
      DebugPrint('\t');
      DebugPrintln((rightdiff/8245.81)*(1000000/16667.0));*/
    }
  }
}


static const float TURN_TOL = 5*(PI/180.0); //Rad

void handleTurn() {
  //Serial.println("Turn");
  if(targetTurnHeading>(2*PI))
  {
    targetTurnHeading -= (2*PI);
  }
  float currentHeading = getTheta();
  long long pidOut = abs(turnPid -> compute(targetTurnHeading*10000,currentHeading*10000)); //scaled because pid uses long longs
  if(currentHeading < targetTurnHeading)
  {
    //turn counter clockwise
    lastLeftSetpoint = constrain(-1*pidOut,-1*MAX_SETPOINT,MAX_SETPOINT);
    lastRightSetpoint = constrain(pidOut,-1*MAX_SETPOINT,MAX_SETPOINT);
  }
  else
  {
    //turn clockwise
    lastLeftSetpoint = constrain(pidOut,-1*MAX_SETPOINT,MAX_SETPOINT);
    lastRightSetpoint = constrain(-1*pidOut,-1*MAX_SETPOINT,MAX_SETPOINT);
  }
  DebugPrint((long)lastLeftSetpoint);
  DebugPrint('\t');
  DebugPrint((long)lastRightSetpoint);
    DebugPrint('\t');
  DebugPrintln(abs(getTheta()-targetTurnHeading));
  if(abs(getTheta()-targetTurnHeading)<TURN_TOL)
  {
      lastLeftSetpoint = 0; //stop motors
      lastRightSetpoint = 0;
      currentState = previousState;
      previousState = TURN;
      Serial.println("Back to wall");
  }
}
