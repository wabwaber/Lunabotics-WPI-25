#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class HBridge {
 public:

  HBridge();

  void init(int, int);

  void setEffort(int effort);

 private:
    bool leftSide;
    int L_PWM;
    int R_PWM;
};