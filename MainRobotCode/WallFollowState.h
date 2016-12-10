#ifndef WALL_FOLLOW_STATE_H
#define WALL_FOLLOW_STATE_H
#include "Arduino.h"
#include "DriveController.h"
class WallFollowState {
public:
  static WallFollowState* getInstance() {
    static WallFollowState instance;
    return &instance;
  }
  void handle(); //called from main code
private:
  WallFollowState();
  long long lastLoopTime;
  long long stoppedTime;
  bool startTurn;
  long long walloffsetpidOut = 0;
  long long wallthetapidOut = 0;
  PID* wallOffsetPid;
  PID* wallThetaPid;
  
  static const long long WALL_OFFSET_SETPOINT = (long)10*25.4;
};
#endif
