#include "PID.h"
#include "PreciseTime.h"
PID::PID(unsigned long kp_new,unsigned long ki_new,unsigned long kd_new,unsigned long* input_new,unsigned long* output_new,unsigned long setpoint_new) {
i_sum = 0;
kp = kp_new;
ki = ki_new;
kd = kd_new;
input = input_new;
output = output_new;
setpoint = setpoint_new;
lastError = 0;
lastTime =  preciseTime(); //in 1/16us ticks
}
PID::~PID() {
}
void PID::compute() {
  //get time diff
  static unsigned long long newTime;
  newTime = preciseTime();
  static unsigned long long diff;
  diff = (unsigned long)(newTime - lastTime);
  //get error
  static unsigned long error;
   error = *input-setpoint;
  //compute kp
  static unsigned long newOutput;
  newOutput = error*kp;
  //compute ki
  i_sum += (error*diff);
  newOutput += (i_sum*ki) ;
  //compute kd
  newOutput += kd*(error - lastError)/diff;
  *output = newOutput;  
}
void PID::setSetpoint(unsigned long setpoint_new){ //  also calls reset
  setpoint = setpoint_new;
  reset();
}
void PID::reset() { //clears i term
  i_sum = 0;
  lastError = 0;
  lastTime = preciseTime();
}
