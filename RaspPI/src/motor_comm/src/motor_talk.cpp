// Motor_talk should send out motor driver commands and recieve encoder reads
#include "motor_talk.hpp"


MotorCommunicator::MotorCommunicator(){
        
}

bool MotorCommunicator::setSpeed(motors toChange, uint16_t givenSpeed){
    bool pass = true; //assume true until proven otherwise

    this->requestedSpeed.at(toChange) = givenSpeed;
    
    if(this->requestedSpeed.at(toChange) != givenSpeed){pass = false;} //sanity check in case the setspeed fails for whatever reason

    return pass; //return the bool
}

//below here until update() is just getters for the 3 private variables in case they are needed elsewhere

uint16_t MotorCommunicator::getSetSpeed(motors motor){
    return this->requestedSpeed.at(motor);
}

uint16_t MotorCommunicator::getSpeed(motors motor){
    return this->currSpeed.at(motor);
}

uint16_t MotorCommunicator::getPrevSpeed(motors motor){
    return this->prevSpeed.at(motor);
}


void MotorCommunicator::update(){
    for(int iter = DRIVE_FRONT_LEFT; iter != END_OF_LIST; iter++){ //I know I should be using a switch here but because I don't want to add another switch case each time a new motor is added I just use value undefined enum to iterate through each motor
        //do the PID updates here
        
    }
}