#include "./Talon.h"

Talon::Talon(){}
//Taken from last years code (2024), changed it to work with the slightly different servo.h but otherwise the same

void Talon::init(int PWMPin, bool reverse){
    attached = false;
    pin = PWMPin;
    reversed = reverse;
    currentEffort = 0;
}

void Talon::setEffort24(int effort){
    // Reverese effort if reversed flag is set to true
    if (reversed) {
        effort = -effort;
    }
    //constrain the value between -100 and 100
    if (effort > 100) {
        effort = 100;
    } else if (effort < -100) {
        effort = -100;
    }
    if(abs(effort) < 1){ //if we are given a decimal value that is less than one, it is a request to detach or we've hit our center for PID
        if(attached){  //if we are currently attached
            PWMController.detach(); //detach the controller
            attached = false; //change the bool
        }
    }
    else{ //otherwise
        if(!attached){ //if we are not attached
            PWMController.attach(pin); //attach
            attached = true; //change bool
        }
        PWMController.write(effort); //then write the effort to the motor (servo)
    }
}