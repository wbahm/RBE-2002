#ifndef TURN_STATE_H
#define TURN_STATE_H
#include "Arduino.h"
#include "PID.h"
class TurnState {
public:
static TurnState* getInstance() {
  static TurnState instance;
  return &instance;
}
void handle();
void setTurnHeading(float newHeading);
private:
TurnState();
#define TURN_TOL 1*(PI/180.0) //Rad
float targetTurnHeading; //Rad - based on field abs heading
PID* turnPid;

};
#endif

