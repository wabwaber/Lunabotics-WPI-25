/*
 * intake.cpp
 * 
 *  Created on: Mar 20, 2024
 *      Author: WigglyWalrus
 */

#include "conveyor.h"

Conveyor::Conveyor(){}

void Conveyor::init(){
    can_controller.init();

    plungeMotor.init(PLUNGE_TALON_PWM, true);

    pinMode(PLUNGE_BOT, INPUT_PULLUP);
    pinMode(PLUNGE_TOP, INPUT_PULLUP);

    plungeSpeed = 0.0;
    prevEffort = 0;
    effort = 0;
    atTop = false;
    atBot = false;
    conveyorCurrent = 0;

    enabled = false;
}

void Conveyor::enable() {
    enabled = true;
}

void Conveyor::disable() {
    enabled = false;
}

bool Conveyor::isEnabled() {
    return enabled;
}

bool Conveyor::isAtTop(){
    return atTop;
}

bool Conveyor::isAtBot(){
    return atBot;
}



void Conveyor::loop() {

    // Always Get Data
    can_controller.updateMotorSpeeds();

    can_controller.setMotorCurrent(conveyorCurrent);

    atBot = !(digitalRead(PLUNGE_BOT) == HIGH);
    atTop = !(digitalRead(PLUNGE_TOP) == HIGH);

    // Positive speed means plunging downwards
    // signal goes low when a limit is hit
    if (plungeSpeed > 0 && !atBot){
        // Run motor to plunge down
        effort = plungeSpeed;
    }
    else if(plungeSpeed < 0 && !atTop){
        // Run motor to plunge down
        effort = plungeSpeed;
    }
    else{
        effort = 0;
    }

    plungeMotor.setEffort12(effort);
    // plungeMotor.setEffort12Slow(effort);

}

void Conveyor::setPlungeSpeed(int speed){
    plungeSpeed = speed;
}

float Conveyor::getPlungeSpeed(){
    return plungeSpeed;
}

// void Conveyor::setSpeed(float newSpeed){
//     can_controller.setSpeed(newSpeed);
// }

void Conveyor::setConveyorCurrent(int current){
    conveyorCurrent = current;
}

float Conveyor::getConveyorSpeed(){
    return can_controller.getRealSpeed();
}

int Conveyor::getConveryorCurrent(){
    return can_controller.getCurrent();
}

float Conveyor::getRawAngle(){
    return float(can_controller.getRawAngle());
}