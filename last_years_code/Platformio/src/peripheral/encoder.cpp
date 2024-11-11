/*
 * encoder.cpp
 * 
 *  Created on: Dec 11, 2023
 *      Author: Speeep
 */

#include "encoder.h"
#include <Wire.h>

Encoder::Encoder(){}

void Encoder::init(int id, int multiplexerId, float start) {
  rawMasterEncoder.init(id, multiplexerId);
  startAngle = start;
}

float Encoder::getAngle()
{
  int rawAngle = rawMasterEncoder.getRawAngle();

  radAngle = static_cast<float>(rawAngle) * BYTES_2_RAD - startAngle;

  if (radAngle < -3.14159)
  {
    radAngle += 6.28319;
  }
  
  if (radAngle > 3.14159)
  {
    radAngle -= 6.28319;
  }

return radAngle;
}