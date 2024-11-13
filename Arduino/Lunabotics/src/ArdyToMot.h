#ifndef ARDYTOMOT_H
#define ARDYTOMOT_H

#include <Arduino.h>
#include "MCP2515.h"
#include "Talon.h"


#define LEFT_TURN_PWM_PIN 4
#define RIGHT_TURN_PWM_PIN 3

#define MAX_TURN_VAL 1.5708 //taken from last years code, making it the max turn value as it is what is used in the point turn state in drivetrain.cpp
#define MAX_MOTOR_CURRENT_DRIVE 12000

class ArdyToMot{
    public:
        //Function Prototypes: (see cpp file for details)
        bool setMotors(int LF, int LB, int RF, int RB);
        bool setMotors(int L, int R);
        bool setMotors(int A);
        void init();
        bool setTurn(float L, float R);
        bool setTurn(float B);
};
#endif
