#include <digitalIOPerformance.h>
#include "DrivePWM.h"
#include "PID.h"
#include "Encoder.h"
Encoder leftEnc(20,19);
Encoder rightEnc(10,3);
unsigned long long lastPrinted = 0;
const long long printDelay = 500;
unsigned long long lastContr = 0;
const unsigned long long contrFreq = 200; //hz
const unsigned long long contrT = 1000000/contrFreq;

PID* leftPid;
long lastPwr = 0;
unsigned long long lastEnc = 0;

void setup() {
  // put your setup code here, to run once:
  initDrivePWM(); //enables pid
  Serial.begin(115200);
  Serial.println("Started");
  leftPid = new PID(100,0,5);
}
static const long long UM_PER_TICK = 27;
static const long long US_TO_SEC = 1000000;
static const long long SETPOINT = 100000; // um/sec -> 3.9 in/sec -> 0.46 rps
void loop() {
  // put your main code here, to run repeatedly:
  if((micros()-lastContr)>contrT)
  {    
    //Get time diff and enc diff
    long long diffTime = micros()-lastContr;
    lastContr = micros();
    long long diffTicks = leftEnc.read()-lastEnc;
    lastEnc = leftEnc.read();
    //compute velocity
    long long vel = ((diffTicks*US_TO_SEC*UM_PER_TICK)/diffTime); //in mm/sec
    //compute PID
    long long leftpidOut = leftPid -> compute(SETPOINT,vel);
    //set output
    lastPwr += leftpidOut;
    lastPwr = constrain(lastPwr,-65535,65535);
    if(lastPwr > 0)
    {
      setDrivePWM((unsigned int)lastPwr, LEFT,FORWARD);
    }
    else
    {
      setDrivePWM((unsigned int)(-1*lastPwr), LEFT,BACKWARD);
    }
    if((micros()-lastPrinted)>printDelay)
    {
      lastPrinted = micros();
      /*Serial.print((long)diffTicks);
      Serial.print('\t');
      Serial.print((long)diffTime);
      Serial.print('\t');
      Serial.print((long)vel);
      Serial.print('\t');
      Serial.println((long)lastPwr);*/
    }
  }
 // delay(1000);
}
