/*#include "DFRCam.h"
#include <Wire.h>
DFRCamera::DFRCamera() {
}
void DFRCamera::initCamera() {
  Wire.begin();
             //reg,data
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10); //set sensitivity block
  Write_2bytes(0x33,0x33); delay(10);
  delay(100);
}
Target[4] DFRCamera::getTargets() { //returns all four targets even if invalid
  
}
void DFRCamera::Write_2bytes(byte reg, byte data)
{
    Wire.beginTransmission(slaveAddress);
    Wire.write(d1);
    Wire.write(d2);
    Wire.endTransmission();
}*/
