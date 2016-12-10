#include "RobotOdometry.h"
#include "Arduino.h"

void computeOdometry(Encoder* leftEnc,Encoder* rightEnc) { //called as fast as possible, it handles timing
  if((micros()-myPosition.lastOdometryTime) > ODOMETRY_PERIOD_US) //controls update rate
  {
    myPosition.lastOdometryTime = micros();
    long leftReading = leftEnc->read();
    long rightReading = rightEnc->read(); 
    long leftDiff = leftReading-myPosition.lastLeftEnc;
    long rightDiff = rightReading-myPosition.lastRightEnc;
    myPosition.lastLeftEnc = leftReading;
    myPosition.lastRightEnc = rightReading;
    float leftDist = leftDiff*LEFT_DIST_PER_TICK;
    float rightDist = rightDiff*RIGHT_DIST_PER_TICK;
    
    float distTraveled = (leftDist+rightDist)/2.0;
    myPosition.x_loc += sin((PI/2)-(myPosition.theta/1000.0))*distTraveled;
    myPosition.y_loc += -1*cos((PI/2)-(myPosition.theta/1000.0))*distTraveled;
    
    float encDeltaTheta = ((rightDist-leftDist)/WHEEL_BASE)*1000; //mRad
    //Serial.println(myPosition.theta);
    myPosition.theta += encDeltaTheta; //add gyrodometry here
    if(myPosition.theta > (2*PI*1000))
      myPosition.theta = myPosition.theta-(2*PI*1000);
    
  }
}

long getXLoc() {//Returns value in mm
  return (myPosition.x_loc/1000.0f);
}
long getYLoc() {//Returns value in mm
  return (myPosition.y_loc/1000.0f);
}
float getTheta() {//Returns value in Rad
  return (myPosition.theta/1000.0f);
}
