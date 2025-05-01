#pragma once

#include "Arduino.h"
#include "Servo.h"

class Talon {
 public:

  Talon();

  void init(int, bool);

  void setEffort24(int); // Takes -100-100

 private:
    bool attached;
    Servo PWMController;
    int pin;
    bool reversed;
    int increment;
    int currentEffort;
};