#ifndef DFRCAM_H
#define DFRCAM_H

#include "Arduino.h"

class DFRCamera {
public:
DFRCamera();
void initCamera();
struct Target {
  int xLoc; //pixel
  int yLoc;
  int xSize;
  int ySize;
  int size;
  int xMin;
  int yMin;
  int xMax;
  int yMax;
  int intensity;
  bool validTarget;
};
Target[4] getTargets(); //returns all four targets even if invalid
private:
void Write_2bytes(byte reg, byte data);
#define SENSITIVITY_BLOCK_TWO_REG 0x1A
#define SENSITIVITY_BLOCK_ONE_REG 0x00
#define MODE_REG 0x33
#define UPDATE_REG 0x30
#define ALLOW_CHANGES 0x01
#define ENABLE_CHANGES 0x08
#define ENABLE_REG 0x1A
#define ENABLE_CODE 0x04
};

#endif
