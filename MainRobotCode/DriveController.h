#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H
#include "Arduino.h"
#include "Encoder.h"
#include "PID.h"
class DriveController {
public:
  static DriveController* getInstance() {
    static DriveController instance;
    return &instance;
  }
  //Encoder access wrappers
  long getLeftEncCount();
  long getRightEncCount();
  void update(); //called each time in loop
  void setup(); //called during setup
  void setLeftSetpoint(long newLeft);
  void setRightSetpoint(long newRight);
  void setBothSetpoints(long newLeft,long newRight);
  void addToLeftSetpoint(long addLeft);
  void addToRightSetpoint(long addRight);
  void addToBothSetpoints(long addLeft,long addRight);
  void stop(); //sets both setpoints to zero
  static const long long DEFAULT_SETPOINT = 0.75*2.75*PI*2.54*10000; //0.75 RPS
  static const long long MAX_SETPOINT = 0.9*2.75*PI*2.54*10000; //0.9 RPS
  static const long long MIN_SETPOINT = 0.3*2.75*PI*2.54*10000; //0.3 RPS
private:
  DriveController();
  Encoder* leftEnc;
  Encoder* rightEnc;
  PID* leftPID;
  PID* rightPID;
  long leftSetpoint;
  long rightSetpoint;
};

#endif
