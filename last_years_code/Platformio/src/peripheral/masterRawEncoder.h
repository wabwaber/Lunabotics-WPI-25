#pragma once

#include "Arduino.h"
#include <I2C.h>
#include "../robotMap.h"

class RawMasterEncoder {
 public:

  RawMasterEncoder();

  void init(int id, int multiplexerId);

  int getRawAngle();

 private:
    byte lowbyte;   // raw angle 7:0
    byte highbyte; // raw angle 7:0 and 11:8
    int out; // output
    int encoderNumber;
    int multiplexerNumber;
};