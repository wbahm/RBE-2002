#include <digitalIOPerformance.h>
#include "DrivePWM.h"
#include "WallMath.h"
#include "PID.h"
#include "Encoder.h"

#define DEBUG_PRINT //used to enable debug print statements
#include "debug.h"
Encoder leftEnc(20,19);
Encoder rightEnc(18,3);
unsigned long long lastContr = 0;
const unsigned long long contrFreq = 200; //hz
const unsigned long long contrT = 1000000/contrFreq;

static const long long UM_PER_TICK = 27;
static const long long US_TO_SEC = 1000000;
static const long long SETPOINT = 0.75*2.75*PI*2.54*10000; // um/sec -> 3.9 in/sec -> 0.46 rps
static const long long WALL_SETPOINT = (long)8*25.4;

PID* leftPid;
PID* rightPid;
PID* wallPid;
long lastLeftPwr = 0;
long lastRightPwr = 0;
unsigned long long lastLeftEnc = 0;
unsigned long long lastRightEnc = 0;
void setup() {
  DebugBegin();
  DebugPrintln("Serial started");
  wallInit();
  initDrivePWM();//sets up timer for PWM generation
  leftPid = new PID(100,0,0);
  rightPid = new PID(100,0,0);
  wallPid = new PID(6000000,0,0);//5000000
}
unsigned long long lastWallPID = 0;
long long lastLeftSetpoint = SETPOINT;//+50000;
long long lastRightSetpoint = SETPOINT;//-50000;
long long wallpidOut = 0;
long long MAX_SETPOINT = 0.9*2.75*PI*2.54*10000;
long long MIN_SETPOINT = 0.3*2.75*PI*2.54*10000;
void loop() {
  // wall pid
  if((micros()-lastWallPID) >(1000000/10)) //60 hz
  {
    lastWallPID = micros();
    WallState newState = getWallState(RIGHT_WALL);
    wallpidOut = wallPid -> compute(WALL_SETPOINT,newState.wallDist);
    lastLeftSetpoint = (SETPOINT-wallpidOut);
    lastRightSetpoint = (SETPOINT+wallpidOut);
    lastLeftSetpoint = constrain(lastLeftSetpoint,MIN_SETPOINT,MAX_SETPOINT);
    lastRightSetpoint = constrain(lastRightSetpoint,MIN_SETPOINT,MAX_SETPOINT);
    DebugPrint(newState.wallDist/25.4);
    DebugPrint('\t');
    DebugPrint((long)wallpidOut);
    DebugPrint('\t');
    DebugPrint((long)lastLeftSetpoint);
    DebugPrint('\t');
    DebugPrint((long)lastRightSetpoint);
    DebugPrint('\t');
    static long lastLeftVal = leftEnc.read();
    long leftdiff = leftEnc.read()-lastLeftVal;
    lastLeftVal = leftEnc.read();
    static long lastRightVal = rightEnc.read();
    long rightdiff = rightEnc.read()-lastRightVal;
    lastRightVal = rightEnc.read();
    DebugPrint(leftdiff/8245.81);
    DebugPrint('\t');
    DebugPrintln(rightdiff/8245.81);
  }
  //lastLeftSetpoint = (MAX_SETPOINT-MIN_SETPOINT)/2;
  //lastRightSetpoint = MIN_SETPOINT;
  handleMotorControl(lastLeftSetpoint,lastRightSetpoint);
  //handleMotorControl(30000,30000);
  //delay(15);
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
