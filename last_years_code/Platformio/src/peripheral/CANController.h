#pragma once

#include "Arduino.h"
#include <mcp2515.h>
#include <SPI.h>
#include "masterRawEncoder.h"
#include "../robotMap.h"
#include "medianFilter.h"

class CANController {
public:
    CANController();

    void init();

    void setMotorCurrent();

    void speedHandlerPID();

    void updateMotorSpeeds();

    void setSpeed(float, float, float, float);

    float getSpeed(int);

    float getRealSpeed(int);

    int getDisplacement(int);

    void cutCurrent();

    String getSums();

private:
    struct can_frame canMsgOut;
    RawMasterEncoder motor0Encoder;
    RawMasterEncoder motor1Encoder;
    RawMasterEncoder motor2Encoder;
    RawMasterEncoder motor3Encoder;
    MedianFilter mf0;
    MedianFilter mf1;
    MedianFilter mf2;
    MedianFilter mf3;
    MCP2515 mcp2515;
    long lastTime;
    float prevAngles[4];
    float errors[4];
    float prevErrors[4];
    float setSpeeds[4];
    float speeds[4];
    float realSpeeds[4];
    int filterPosition;
    float sums[4];
    int setCurrents[4];
    float speedSetpoints[4];
    int displacements[4];
};