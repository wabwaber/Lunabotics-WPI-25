#include <ArdyToMot.h>

//Making the rather bold assumption that the wheel numbers goes from left front in a counter Clockwise U. This is because I know the first two are left and the last 2 are right but not which one is front or back
//Function takes in the SPEED for each wheel and tells the motors to work. Returns true if the writing (CAN Message transmission) was successful and false in any other senario
bool ArdyToMot::setMotors(int LF, int LB, int RB, int RF){
    //calculate each drive motors speed
    LF = constrain(LF, -MAX_MOTOR_SPEED_DRIVE, MAX_MOTOR_SPEED_DRIVE);
    LB = constrain(LB, -MAX_MOTOR_SPEED_DRIVE, MAX_MOTOR_SPEED_DRIVE);
    RF = constrain(RF, -MAX_MOTOR_SPEED_DRIVE, MAX_MOTOR_SPEED_DRIVE);
    RB = constrain(RB, -MAX_MOTOR_SPEED_DRIVE, MAX_MOTOR_SPEED_DRIVE);

    //setup message to be sent for drive motors
    currmsg.adrsValue = CAN_ID_Drive;
    currmsg.isExtendedAdrs = false ? Drive_DLC==8 : true;
    currmsg.rtr = false;
    currmsg.dataLength = Drive_DLC;
    
    //write packet data
    currmsg.data[0] = (byte)(LF / ConversionFactor);
    currmsg.data[1] = (byte)(LF % ConversionFactor);
    currmsg.data[2] = (byte)(LB / ConversionFactor);
    currmsg.data[3] = (byte)(LB % ConversionFactor);
    currmsg.data[4] = (byte)(RB / ConversionFactor);
    currmsg.data[5] = (byte)(RB % ConversionFactor);
    currmsg.data[6] = (byte)(RF / ConversionFactor);
    currmsg.data[7] = (byte)(RF % ConversionFactor);

    //send it
    return can.transmitCANMessage(currmsg, 1000); //msg that is being transmitted and the timeout period
}

//Function takes in SPEED for the left side and the right side. And like the one above returns true if OK and false in any other senario
bool ArdyToMot::setMotors(int L, int R){
    return setMotors(L, L, R, R);
}

bool ArdyToMot::setMotors(int A){
    return setMotors(A, A, A, A);
}

bool ArdyToMot::setTurn(float L, float R){
    //disabled because its not wired
    PWMLeft.setEffort24(L);
    PWMRight.setEffort24(R);
    return true; //mhm we 100% for sure set the efforts
} 

bool ArdyToMot::setTurn(float B){
    return setTurn(B, B);
}

//Initializer function for the arduino to motor communication.
//Just starts CAN communication.
void ArdyToMot::init(){
    SPI.setClockDivider(SPI_CLOCK_DIV8); //taken from can bus example
    if(can.initCAN(CAN_BAUD_100K)==0){
        Serial.println("initCAN() failed");
        exit(1); //exit
    }

    if(can.setCANNormalMode(LOW) == 0){
        Serial.println("setCANNormalMode() failed");
        exit(1);
    }

    //turnErrorLeft = 0;
    //turnErrorRight = 0; 
    setMotors(0); //set ALL motors to 0 once the CAN has begun
    //setTurn(0); //zero the turn angles
    // PWMLeft.init(LPWM, false);
    // PWMRight.init(RPWM, false);
    return;
}

double ArdyToMot::getDriveInfo(){ 
    return (double) can.queryOBD(CAN_ID_Drive);
}

double ArdyToMot::getConveyInfo(){
    return (double) can.queryOBD(CAN_ID_Conveyor);
}