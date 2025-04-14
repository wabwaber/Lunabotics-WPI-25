// Motor_talk should send out motor driver commands and recieve encoder reads
#include "motor_talk.hpp"


MotorController::MotorController(){
        
}

bool MotorController::setSpeed(motors toChange, uint16_t givenEffort){
    bool pass = true; //assume true until proven otherwise

    this->requestedSpeed.at(toChange) = givenEffort;
    
    if(this->requestedSpeed.at(toChange) != givenEffort){pass = false;} //sanity check in case the setspeed fails for whatever reason

    return pass; //return the bool
}

//below here until update() is just getters for the 3 private variables in case they are needed elsewhere

uint16_t MotorController::getSetSpeed(motors motor){
    return this->requestedSpeed.at(motor);
}

uint16_t MotorController::getSpeed(motors motor){
    return this->currSpeed.at(motor);
}

uint16_t MotorController::getPrevSpeed(motors motor){
    return this->prevSpeed.at(motor);
}


void MotorController::update(){
    for(int iter = LEFT_TURN; iter != END_OF_LIST; iter++){ //I know I should be using a switch here but because I don't want to add another switch case each time a new motor is added I just use value undefined enum to iterate through each motor
        //do the PID updates here
        
    }
}