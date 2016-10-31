#include "DrivePWM.h"
void setup() {
  initDrivePWM();
}

void loop() {
    setDrivePWM(65535*0.75,LEFT);
    setDrivePWM(65535*0.5,RIGHT);
    delay(500);
}
