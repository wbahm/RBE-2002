#ifndef DRIVE_CONTROLLER_H
#define DRIVE_CONTROLLER_H

class DriveController {
public:
  //Encoder access wrappers
  long getLeftEncCount();
  long getRightEncCount();
private:
  Encoder* leftEnc;
  Encoder* rightEnc;
};

#endif
