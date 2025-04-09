#include "./encoder_talk.hpp"
//#include "./msg/EncoderRead.msg"
#include "Talon.h"
#include <unordered_map>
#include <string>

//updates these pins as needed
#define LEFT_TURN_PWM_PIN 4
#define RIGHT_TURN_PWM_PIN 3
#define DEPOSIT_PWM 5
#define PLUNGE_PWM 7
#define PLUNGE_MOTOR_EFFORT 30

//velocity loop PID parameters (from last years code)
#define BASE_CURRENT 10
#define SPEED_KP 750
#define SPEED_KI 42
#define SPEED_KD 0
#define SPEED_SUMCAP 380
#define POS_KP 0.05

#define MAX_TURN_VAL 1.5708 //taken from last years code
#define MAX_MOTOR_SPEED_DRIVE 12000

static const uint8_t CAN_CS = 41; //SPI Chip Select pin
static const uint8_t CAN_SI = 37; //SPI Data input pin
static const uint8_t CAN_SO = 39; //SPI Data output pin
static const uint8_t CAN_SCK = 35; //SPI Clock input pin
static const uint8_t CAN_INT = 2; //Interrupt pin
int CAN_ID_Drive = 0x200; //512, Drivetrain CAN system ID
int CAN_ID_Conveyor = 0x1FF; //511, Conveyor CAN system ID
int Drive_DLC = 8; //Length of CAN message for drivetrain motors
int Conveyor_DLC = 8; //Length of CAN message for conveyor motors
int ConversionFactor = 256;
float turnErrorLeft;
float turnErrorRight;
float turnSetAngleLeft;
float turnSetAngleRight;
Talon PWMLeft;
Talon PWMRight;
// MCP2515 can;
// CANMSG currmsg;

//All references of direction are from the robots POV IE where is the camera (realsense) pointing
//used for motor speed setting, 0 for CAN bus, 1 for PWM 
enum motorType {DRIVE_FL = 0, DRIVE_FR = 0, DRIVE_BL=0, DRIVE_BR=0, TURN_LEFT=1, TURN_RIGHT=1, DEPOSIT=1, INTAKE_VERT=1, INTAKE_RUN=1};
enum motors {DRIVE_FRONT_LEFT, DRIVE_FRONT_RIGHT, DRIVE_BACK_LEFT, DRIVE_BACK_RIGHT, DEPOSIT, END_OF_LIST}; //this enum exists so there is a precompile list of motors to use and can be referenced by other blocks of code also END_OF_LIST is there for iterators
class MotorCommunicator{
    public:
        MotorCommunicator();
        bool setSpeed(motors toChange, uint16_t givenSpeed);    //returns a bool, true for set success, false for failure or error
        uint16_t getSetSpeed(motors motor);   //returns the set speed, requires the name for the motor
        uint16_t getSpeed(motors motor);     //returns current speed,  ^
        uint16_t getPrevSpeed(motors motor);//returns the previous speed,^
        void update(); //PID loop for motors reaching their requested speed

    private:
        //all un ordered hashmaps as there are a lot of motors. the string takes the exact same name as the Enum at the top not using it though as its values are the same between
        std::unordered_map<motors, uint16_t> currSpeed;        //in RPM and refering to the motor speed not output shaft.
        std::unordered_map<motors, uint16_t> prevSpeed;       //^
        std::unordered_map<motors, uint16_t> requestedSpeed; //^
};