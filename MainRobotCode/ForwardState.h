#ifndef FORWARD_STATE_H
#define FORWARD_STATE_H
#include "Arduino.h"
#include "DriveController.h"
class ForwardState {
public:
static ForwardState* getInstance() {
  static ForwardState instance;
  return &instance;
}
void handle();
void setForwardTarget(long long newTarget);
private:
ForwardState();
bool started;
long startX; //mm
long startY; //mm
long long stopTime;
bool stopping;
long long forwardTarget;//mm dist

//Constants
static const int FORWARD_TOL = 0.5*25.4;//mm
};

#endif
