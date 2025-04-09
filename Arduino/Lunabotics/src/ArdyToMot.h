#ifndef ARDYTOMOT_H
#define ARDYTOMOT_H

#include <Arduino.h>
#include "MCP2515.h" //can bus for drive
#include "Talon.h" //turn motors
#include <Wire.h> //I^2C lib for encoders
#include <SPI.h>


#define LEFT_TURN_PWM_PIN 4
#define RIGHT_TURN_PWM_PIN 3

#define MAX_TURN_VAL 1.5708 //taken from last years code, making it the max turn value as it is what is used in the point turn state in drivetrain.cpp
#define MAX_MOTOR_SPEED_DRIVE 12000 //RPM

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
int ConversionFactor = 256;
float turnErrorLeft;
float turnErrorRight;
float turnSetAngleLeft;
float turnSetAngleRight;
Talon PWMLeft;
Talon PWMRight;
MCP2515 can;
CANMSG currmsg;

class ArdyToMot{
    public:
        //Function Prototypes: (see cpp file for details)
        bool setMotors(int LF, int LB, int RF, int RB);
        bool setMotors(int L, int R);
        bool setMotors(int A);
        void init();
        bool setTurn(float L, float R);
        bool setTurn(float B);
        double getDriveInfo();
        double getConveyInfo();
};
#endif
