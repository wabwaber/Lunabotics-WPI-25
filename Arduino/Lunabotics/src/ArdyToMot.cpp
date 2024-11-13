#include <ArdyToMot.h>


static const int LPWM = 10;
static const int RPWM = 11;
static const uint8_t CAN_CS = 41; //SPI Chip Select pin
static const uint8_t CAN_SI = 37; //SPI Data input pin
static const uint8_t CAN_SO = 39; //SPI Data output pin
static const uint8_t CAN_SCK = 35; //SPI Clock input pin
static const uint8_t CAN_INT = 2; //Interrupt pin
int CAN_ID_Drive = 0x200; //512, Drivetrain CAN system ID
int CAN_ID_Conveyor = 0x1FF; //511, Conveyor CAN system ID
int Drive_DLC = 8; //Length of CAN message for drivetrain motors
int Conveyor_DLC = 8; //Length of CAN message for conveyor motors
int ConversionFactor = 128;
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_1000kbps;
float turnErrorLeft;
float turnErrorRight;
float turnSetAngleLeft;
float turnSetAngleRight;
Talon PWMLeft;
Talon PWMRight;
MCP2515 mcp2515;

//Making the rather bold assumption that the wheel numbers goes from left front in a counter Clockwise U. This is because I know the first two are left and the last 2 are right but not which one is front or back
//Function takes in the current for each wheel and tells the motors to work. Returns true if the writing (CAN Message transmission) was successful and false in any other senario
bool ArdyToMot::setMotors(int LF, int LB, int RB, int RF){
    LF = constrain(LF, -MAX_MOTOR_CURRENT_DRIVE, MAX_MOTOR_CURRENT_DRIVE);
    LB = constrain(LB, -MAX_MOTOR_CURRENT_DRIVE, MAX_MOTOR_CURRENT_DRIVE);
    RF = constrain(RF, -MAX_MOTOR_CURRENT_DRIVE, MAX_MOTOR_CURRENT_DRIVE);
    RB = constrain(RB, -MAX_MOTOR_CURRENT_DRIVE, MAX_MOTOR_CURRENT_DRIVE);
    const int packet[] = {
        (char)(LF / ConversionFactor), (char)(LF % ConversionFactor),
        (char)(LB / ConversionFactor), (char)(LB % ConversionFactor),
        (char)(RB / ConversionFactor), (char)(RB % ConversionFactor), 
        (char)(RF / ConversionFactor), (char)(RF % ConversionFactor)
    };
    CANFrame yeet(CAN_ID_Drive, packet, Drive_DLC);
    CANController::IOResult res = CAN.write(yeet);
    String set = "";
    for(int i = 0; i < 8; i++){
        set += packet[i];
        set += " ";
    }
    Serial.println(set);
    return CANController::IOResult::OK  == res;
}

//Function takes in current for the left side and the right side. And like the one above returns true if OK and false in any other senario
bool ArdyToMot::setMotors(int L, int R){
    return setMotors(L, L, R, R);
}

bool ArdyToMot::setMotors(int A){
    return setMotors(A, A, A, A);
}

bool ArdyToMot::setTurn(float L, float R){
    //disabled because its not wired
    // PWMLeft.setEffort24(L);
    // PWMRight.setEffort24(R);
    return true; //mhm we 100% for sure set the efforts
} 

bool ArdyToMot::setTurn(float B){
    return setTurn(B, B);
}

//Initializer function for the arduino to motor communication.
//Just starts CAN communication.
void ArdyToMot::init(){
    CAN.begin();
    //turnErrorLeft = 0;
    //turnErrorRight = 0; 
    setMotors(0); //set ALL motors to 0 once the CAN has begun
    //setTurn(0); //zero the turn angles
    // PWMLeft.init(LPWM, false);
    // PWMRight.init(RPWM, false);
    return;
}