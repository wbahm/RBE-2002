#ifndef PID_h
#define PID_h
#include "Arduino.h"
class PID {
public:
PID(long long p,long long i,long long d);
long long compute(long long set,long long act);//returns output term
void reset();
private:
long long kp;
long long ki;
long long kd;
long long lastTime;
long long intError; //integral of error; in error*microseconds
long long lastError;
};
#endif

