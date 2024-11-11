#include "./Talon.h"

Talon::Talon() {}

void Talon::init(int PWMPin, bool reverse){
    attached = false;
    pin = PWMPin;
    reversed = reverse;
    currentEffort = 0;
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