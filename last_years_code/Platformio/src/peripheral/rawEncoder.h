#pragma once

#include "Arduino.h"
#include "../robotMap.h"

class RawEncoder {
 public:

  RawEncoder();

  void init(int id, int multiplexerId);

  int getRawAngle();

 private:
    int lastRawAngle;
    int lowbyte;   // raw angle 7:0
    word highbyte; // raw angle 7:0 and 11:8
    int encoderNumber;
    int multiplexerNumber;
};