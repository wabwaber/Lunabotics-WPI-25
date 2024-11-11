#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/talon.h"
#include "./peripheral/encoder.h"

class Localizer {
    public:

        Localizer();

        void init();

        void enable();

        void disable();

        bool isEnabled();

        void setError(float);

        float getError();

        float getAngle();

        void loop();

    private:

        Talon turnMotor;
        Encoder encoder;
        bool enabled;
        float angle;
        float lastAngle;
        float angleSetpoint;
        float error;
        float errors;
        bool hysteresis;
        bool turnAround;
        bool turnClockwise;
        bool turning;

};