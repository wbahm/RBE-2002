#ifndef ROBOT_ODOMETRY_H
#define ROBOT_ODOMETRY_H
#include "Arduino.h"
#include "Encoder.h"
struct RobotPos {
  long x_loc = 0; //um
  long y_loc = 0; //um
  float theta = 0; //uRad
  long lastLeftEnc = 0;
  long lastRightEnc = 0;
  long long lastOdometryTime = 0; //us
};
static RobotPos myPosition;

static const long long FREQ_ODOMETRY = 200; //hz
static const long long US_PER_ODOMETRY = 1000000/FREQ_ODOMETRY; //in usec

void computeOdometry(Encoder* leftEnc,Encoder* rightEnc); //called as fast as possible, it handles timing

long getXLoc();//Returns value in mm
long getYLoc();//Returns value in mm
float getTheta();//Returns value in mRad

static const float TICKS_PER_REV = 8245.81;
static const float LEFT_DIST_PER_REV = 2.74*25.4*1000*PI;
static const float RIGHT_DIST_PER_REV = 2.74*25.4*1000*PI;
static const float LEFT_DIST_PER_TICK = LEFT_DIST_PER_REV/TICKS_PER_REV;
static const float RIGHT_DIST_PER_TICK = RIGHT_DIST_PER_REV/TICKS_PER_REV;

static const float WHEEL_BASE = 9*25.4*1000; //um check if correct

#endif
