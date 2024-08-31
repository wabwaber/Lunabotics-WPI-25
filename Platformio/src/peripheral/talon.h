#pragma once

#include "Arduino.h"
#include "Servo.h"
#include "../robotMap.h"

class Talon {
 public:

  Talon();

  void init(int, bool);

  void setEffort12(int); // Takes -100-100

  void setEffort24(int); // Takes -100-100

  void setEffort12Slow(int); // Takes -100 to 100, for gradual effort increase

 private:
    bool attached;
    Servo PWMController;
    int pin;
    bool reversed;
    int increment;
    int currentEffort;
};