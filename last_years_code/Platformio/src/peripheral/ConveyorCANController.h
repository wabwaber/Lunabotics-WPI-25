#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "masterRawEncoder.h"
#include "../robotMap.h"
#include "medianFilter.h"

class ConveyorCANController {
public:
    ConveyorCANController();

    void init();

    void setMotorCurrent(int);

    void updateMotorSpeeds();

    float getSpeed();

    float getRealSpeed();

    void cutCurrent();

    String getSums();

    int getCurrent();

    int getRawAngle();

private:
    struct can_frame canMsgOut;
    RawMasterEncoder motor0Encoder;
    MedianFilter mf0;
    MCP2515 mcp2515;
    long lastTime;
    int prevAngle;
    float error;
    float prevError;
    float speedSet;
    float speed;
    float realSpeed;
    int filterPosition;
    float sum;
    int setCurrent;
    float speedSetpoint;
    int displacement;
    int motorCurrent0;
    int motor0Angle;
    int motor0deltaAngle;
    long time;
    long deltaTime;
    float motor0Speed;
};