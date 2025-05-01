#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "masterRawEncoder.h"

class Encoder {
 public:

  Encoder();

  void init(int id, int multiplexerId, float start);

  float getAngle();

 private:
    float startAngle;
    float radAngle;
    float rawAngle;
    RawMasterEncoder rawMasterEncoder;
};