#include "PID.h"

PID::PID(long long p,long long i,long long d) {
  kp = p;
  ki = i;
  kd = d;
  lastTime = micros();
  lastError = 0;
  intError = 0;
}
long long PID::compute(long long set,long long act) {//recalculates output term
  long long diffTime = micros()-lastTime;
  lastTime = micros();
  long long error = set-act;
  long long diffError = ((error-lastError)*1000000)/diffTime;
  lastError = error;
  intError += (error*diffTime)/1000000;
  #define K_GAIN 10000
  return ((error*kp)/K_GAIN)+((intError*ki)/K_GAIN)+((diffError*kd)/K_GAIN);
}
void PID::reset() {
  intError = 0;
  lastError = 0;
}

