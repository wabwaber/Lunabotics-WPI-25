#include <Arduino.h>
#include <ArdyToMot.h>
#include <stdlib.h>
#include <string.h>

ArdyToMot DriveMotor;
enum States : int {AllStop = 0, Drive = 1, TurnOnPoint = 2, TurnAroundPoint = 3};
States currState;
int mult = 2 * MAX_MOTOR_CURRENT_DRIVE;
bool isSerialReady = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //start serial with BAUD of 115200
  DriveMotor.init();
}

void loop() {
  // put your main code here, to run repeatedly:
  String str = "";
  str = Serial.readStringUntil((char)0);
  //Serial.println(str); //debugging line, uncomment for use

  char* nums = (char*)str.c_str();
  char* currNum = strtok(nums, " ");
  float map[6];
  for(int i = 0; i < 6; i++){
     map[i] = atof(currNum);
     currNum = strtok(NULL, " ");
  }

  /* Fancy stuffs that I may free someday, but that day is not today!
  int turnEffort;
  //turning efforts go 100 to -100
  if(map[4] < 0){ //if the value for the left turn is negative
    //do the maths using the right turn value
    turnEffort = 100 * map[5];
    DriveMotor.setTurn(-turnEffort, turnEffort);
  }
  else if(map[5] < 0){//if the value for the right turn is negative
    //do maths using left turn value
    turnEffort = 100 * map[4];
    DriveMotor.setTurn(turnEffort, -turnEffort);
  }
  else{//its zero
    DriveMotor.setTurn(0);//zero it out
  } */

  DriveMotor.setMotors(mult*map[0], mult*map[1], mult*map[2], mult*map[3]); 
}
