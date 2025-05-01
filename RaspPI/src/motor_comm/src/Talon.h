#pragma once
#include <Servo.h>
#include <map>
#include <stdlib.h>

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