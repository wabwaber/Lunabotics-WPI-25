/*
 * snowblower.cpp
 *
 *  Created on: Sep 28, 2021
 *      Author: Speeep
 */

#include "HBridge.h"
#include <Wire.h>

HBridge::HBridge() {}

void HBridge::init(int RPWM, int LPWM)
{
        pinMode(RPWM, OUTPUT);
        pinMode(LPWM, OUTPUT);
        R_PWM = RPWM;
        L_PWM = LPWM;
}

void HBridge::setEffort(int effort)
{
    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }

    if (effort > 0) {
        analogWrite(L_PWM, map(effort, 0, 100, 20, 255));
        analogWrite(R_PWM, 0);
    } else if (effort < 0) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, map(abs(effort), 0, 100, 20, 255));
    } else if (effort == 0) {
        analogWrite(L_PWM, 0);
        analogWrite(R_PWM, 0);
    }
}