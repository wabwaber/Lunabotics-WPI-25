#include <Arduino.h>
#include <ArdyToMot.h>
#include <stdlib.h>

ArdyToMot DriveMotor;
enum States : int {AllStop = 0, Drive = 1, TurnOnPoint = 2, TurnAroundPoint = 3};
States currState;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //start serial with BAUD of 115200
//  while(!Serial){}; //Wait for Serial to open
//nah we are gonna assume that its open right now because fuck it
  DriveMotor.init();
  DriveMotor.setMotors(0); //stop all motors
}

void loop() {
  // put your main code here, to run repeatedly:
  char* in = (char*) Serial.read(); //current input mapping
//FOR THIS ^^^^ : https://stackoverflow.com/questions/57399227/convert-c-byte-array-to-a-c-string
//incase it doesn't work

  int map[6]; //mapping of the input
  char* currNum = strtok(in, " "); //tokenize the numbers
  int i = 0; //index for mapping
  while(currNum != NULL){ //while we are still iterating
    map[i] = atof(currNum); //convert the string to an int
    currNum = strtok(NULL, " "); //get the next number
    i++; //increment the index
  }
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
  }
  DriveMotor.setMotors(map[0], map[1], map[2], map[3]);
}
