#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/talon.h"
#include "./peripheral/encoder.h"

class Deposit {
    public:

        Deposit();

        void init();

        void enable();

        void disable();

        bool isEnabled();

        float getAngle();

        void setOpen(bool);

        bool isOpen();

        bool isInPosition();

        void loop();

    private:

        Talon depositMotor;
        Encoder encoder;
        bool enabled;
        float angle;
        bool open;
        float error;
        float setpoint;
        float effort;

};

