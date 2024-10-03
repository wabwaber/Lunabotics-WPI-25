#include <Arduino.h>
#include <ArdyToMot.h>
#include <stdlib.h>

ArdyToMot DriveMotor;
enum States : int {AllStop = 0, Drive = 1, TurnOnPoint = 2, TurnAroundPoint = 3};
States currState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //start serial with BAUD of 115200
  while(!Serial){}; //Wait for Serial to open
  DriveMotor.init();
  DriveMotor.setMotors(0); //stop all motors
}

void loop() {
  // put your main code here, to run repeatedly:
  float* map = (float *) Serial.read(); //current input mapping
  DriveMotor.setMotors(map[0], map[1], map[2], map[3]);
}

