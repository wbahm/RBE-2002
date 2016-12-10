#ifndef SPEC_CASE_STATE_H
#define SPEC_CASE_STATE_H

#include "Arduino.h"

class SpecCaseState {
public:
  static SpecCaseState* getInstance() {
    static SpecCaseState instance;
    return &instance;
  }
  void handle();
private:
  SpecCaseState();
  
};

#endif
