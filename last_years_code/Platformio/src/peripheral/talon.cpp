/*
 * talon.cpp
 *
 *  Created on: Apr 3, 2024
 *      Author: WigglyWalrus
 */

#include "talon.h"

Talon::Talon() {}

void Talon::init(int PWMpin, bool reverse)
{
    attached = false;
    pin = PWMpin;
    reversed = reverse;
    currentEffort = 0;
    increment = 5;
}

void Talon::setEffort12(int effort)
{
    // Reverese effort if reversed flag is set to true
    if (reversed) {
        effort = -effort;
    }

    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }
    if(abs(effort) < 1){
        if(attached){
            PWMController.detach();
            attached = false;
        }
    }
    else{
        if(!attached){
            PWMController.attach(pin);
            attached = true;
        }
        effort = (int)map(effort,-100,100,45,135);
        PWMController.write(effort);
    }
}
void Talon::setEffort24(int effort)
{

    // Reverese effort if reversed flag is set to true
    if (reversed) {
        effort = -effort;
    }

    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }
    if(abs(effort) < 1){
        if(attached){
            PWMController.detach();
            attached = false;
        }
    }
    else{
        if(!attached){
            PWMController.attach(pin);
            attached = true;
        }
        effort = (int)map(effort,-100,100,0,180);
        PWMController.write(effort);
    }
}

void Talon::setEffort12Slow(int targetEffort)
{    

    // Clamp targetEffort to the -100 to 100 range
    targetEffort = max(-100, min(100, targetEffort));

    // Calculate the next effort step towards the target
    if (currentEffort < targetEffort) {
        currentEffort = min(currentEffort + increment, targetEffort);
    } else if (currentEffort > targetEffort) {
        currentEffort = max(currentEffort - increment, targetEffort);
    }

    // Use setEffort12 to apply the current effort
    setEffort12(currentEffort);
}