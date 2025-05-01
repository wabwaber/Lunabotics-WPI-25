// Motor_talk should send out motor driver commands and recieve encoder reads
#include "motor_talk.hpp"

MotorController::MotorController(){
            //basic init for the PWM motors
            PWMLeft = Talon();
            PWMRight = Talon();
            PWMIntakeRun = Talon();
            PWMIntakeVerticalAndDesposit = Talon();
            is_H_bridge_to_Despoit = false;
            //actually initalize the pins for the motor controllers
            PWMLeft.init(LEFT_TURN_PWM_PIN, false);
            PWMRight.init(RIGHT_TURN_PWM_PIN, false);
            PWMIntakeRun.init(INTAKE_RUN_PWM_PIN, false);
            PWMIntakeVerticalAndDesposit.init(DEPOSIT_AND_VERTICAL_PWM_PIN, false);
            //then add them to the list of motors
            listOfMotors[0] = PWMLeft;
            listOfMotors[1] = PWMRight;
            listOfMotors[2] = PWMIntakeRun;
            listOfMotors[3] = PWMIntakeVerticalAndDesposit; //the last two are the same as they are the Intake Vertical
            listOfMotors[4] = PWMIntakeVerticalAndDesposit; //and Deposit motors repectivly
            wiringPiSetupGpio(); //need to do this to get the GPIO pins available to change them
            pinMode(DEPOSIT_VERTICAL_SWITCH_PIN, OUTPUT);
            digitalWrite(DEPOSIT_VERTICAL_SWITCH_PIN, LOW); //set the switch pin to low by default
}

bool MotorController::setSpeed(motors toChange, uint16_t givenEffort){
    bool pass = true; //assume true until proven otherwise

    this->requestedEffort.at(toChange) = givenEffort;
    
    if(this->requestedEffort.at(toChange) != givenEffort){pass = false;} //sanity check in case the setspeed fails for whatever reason

    return pass; //return the bool
}

//below here until update() is just getters for the 3 private variables in case they are needed elsewhere

uint16_t MotorController::getSetSpeed(motors motor){
    return this->requestedEffort.at(motor);
}

uint16_t MotorController::getSpeed(motors motor){
    return this->currSpeed.at(motor);
}

uint16_t MotorController::getPrevSpeed(motors motor){
    return this->prevSpeed.at(motor);
}

MotorController::~MotorController(){
    free(listOfMotors); //free the memory
}

void MotorController::update(){
    double newEffort = 0;
    for(int iter = LEFT_TURN; iter != END_OF_LIST; iter++){ //I know I should be using a switch here but because I don't want to add another switch case each time a new motor is added I just use value undefined enum to iterate through each motor
        //do the PID updates here
        /* yoiked from the CAN bus PID
        newCurrent = []
        for x in motor:
            self.errors[x] = self.requested_speeds[x] - self.curr_speeds[x]
            self.sums[x] = min(MAX_SPEED_SUM, max(-MAX_SPEED_SUM, self.sums[x] + self.errors[x])) #basically constrains the value between the MAX and MIN currents set above,
            if self.requested_speeds[x] == 0:
                newCurrent.append(0)
                self.sums[x] = 0
            elif self.requested_speeds > 0:
                newCurrent.append(BASE_CURRENT + SPEED_KP * self.errors[x] + SPEED_KI * self.sums[x])
            else:
                newCurrent.append(-BASE_CURRENT + SPEED_KP * self.errors[x] + SPEED_KI * self.sums[x])
        self.prevErrors = deepcopy(self.errors)
        self.set_currents(newCurrent)
        */   
        if(iter == DEPOSIT){ //if we need to switch the switch pin over to the deposit
            digitalWrite(DEPOSIT_VERTICAL_SWITCH_PIN, HIGH); //switch the pin over
        }
        else if(iter == VERTICAL_INTAKE){ //if we need to change the vertical motor
            digitalWrite(DEPOSIT_VERTICAL_SWITCH_PIN, LOW); //ensure the pin is low to select it
        }
        this->errors[iter] = this->requestedEffort.at(motors(iter)) - this->currSpeed.at(motors(iter));
        double sumError = this->sums[iter] + this->errors[iter];
        if(sumError < -MAX_SPEED_SUM){
            sumError = -MAX_SPEED_SUM;
        }
        else if(sumError > MAX_SPEED_SUM){
            sumError = MAX_SPEED_SUM;
        }
        this->sums[iter] = sumError;
        if(this->requestedEffort.at(motors(iter)) == 0){
            this->sums[iter] = 0;
        }
        else if(this->requestedEffort.at(motors(iter)) > 0){ //if its a positive number
            newEffort = KP * this->errors[iter] + KI * this->sums[iter]; //move the motor forwards
        }
        else{ //otherwise
            newEffort = -KP * this->errors[iter] + KI * this->sums[iter]; //move it backwards
        }
        this->listOfMotors[iter].setEffort24(newEffort); //set the new effort
        //then continue again
    }
}
