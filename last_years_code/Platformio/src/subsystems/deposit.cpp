/*
 * deposit.cpp
 * 
 *  Created on: Mar 20, 2024
 *      Author: WigglyWalrus
 */
#include "deposit.h"

Deposit::Deposit(){}

void Deposit::init() {
    depositMotor.init(DEPOSIT_PWM, true);
    encoder.init(DEPOSIT_ENCODER_ID, MULTIPLEXER_1_ID, 0.0);
    enabled = true;
    angle = 0.0;
    open = false;
    error = 0;
    setpoint = 0.0;
    effort = 0.0;
}

void Deposit::enable() {
    enabled = true; 
}

void Deposit::disable() {
    enabled = false;
}

bool Deposit::isEnabled() {
    return enabled;
}

float Deposit::getAngle() {
    return angle;
}

void Deposit::setOpen(bool newOpen){
    open = newOpen;
}

bool Deposit::isOpen(){
    return open;
}

bool Deposit::isInPosition(){
    if(open){
        if(abs(getAngle() - DEPOSIT_OPEN_ANGLE) < DEPOSIT_ANGLE_THRESHOLD){
            return true;
        }
        return false;
    }
    else{
        if(abs(getAngle() - DEPOSIT_CLOSED_ANGLE) < DEPOSIT_ANGLE_THRESHOLD){
            return true;
        }
        return false;
    }
}

void Deposit::loop() {

    // Always get Data
    angle = encoder.getAngle();

    // Don't set effort if close enough
    if (isInPosition()) {
        depositMotor.setEffort24(0);
        return;
    }

    // // Determine Setpoint
    if (open) {
        setpoint = DEPOSIT_OPEN_ANGLE;
    } else {
        setpoint = DEPOSIT_CLOSED_ANGLE;
    }

    // // Calculate Error
    error = setpoint - angle;

    // // If enabled, control the motors, else cut current to the motors
    if (enabled) {

        // Cap effort at 50
        effort = int((error * DEPOSIT_MOTOR_KP));
        if (effort > 50) {
            depositMotor.setEffort24(50);
        } else {
            depositMotor.setEffort24(effort);
        }
    }
    else{
        depositMotor.setEffort24(0);
    }
}