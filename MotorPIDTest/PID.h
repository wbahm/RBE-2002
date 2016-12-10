#ifndef PID_H
#define PID_H

class PID {
public:
// Constructor/destructor
PID(unsigned long kp_new,unsigned long ki_new,unsigned long kd_new,unsigned long * input_new,unsigned long * output_new,unsigned long  setpoint_new) ;
~PID() ;
void compute();
void setSetpoint(unsigned long setpoint_new); //  also calls reset
void reset(); //clears i term
private:
unsigned long i_sum;
unsigned long kp;
unsigned long ki;
unsigned long kd;
unsigned long* input;
unsigned long* output;
unsigned long setpoint;
unsigned long long lastTime;
unsigned long lastError;
};
#endif
