#pragma once

#include "Arduino.h"
#include "../robotMap.h"
#include "./peripheral/talon.h"
#include "./peripheral/encoder.h"
#include "./peripheral/CANController.h"
#include <std_msgs/Float32MultiArray.h>

class Drivetrain {
 public:

  Drivetrain();

  void init();

  void enable();

  void disable();

  void setState(int);

  int getState();

  void loop();

  void setLeftWheelpodAngle(int);

  void setRightWheelpodAngle(int);

  void setWheelSpeeds(float, float, float, float);

  void setWheelSpeeds(float, float);
  
  void turnICC(float, float);

  float getSpeed(int);

  float getRealSpeed(int);

  float getLeftWheelpodAngle();

  float getRightWheelpodAngle();

  void setLeftWheelpodAngleSetpoint(float);

  void setRightWheelpodAngleSetpoint(float);

  float getLeftWheelpodAngleSetpoint();

  float getRightWheelpodAngleSetpoint();

  void setAngle(bool);

  bool isEnabled();

  void setDriveSpeed(float);

  void setYICC(float);

  float getDriveSpeed();

  String getSums();

  void stepOdom();

  float getPoseStepX();

  float getPoseStepY();

  float getPoseStepTheta();


 private:
    Talon left_turn_motor;
    Talon right_turn_motor;
    Encoder left_wheelpod_encoder;
    Encoder right_wheelpod_encoder;
    CANController can_controller;
    float leftWheelpodAngleSetpoint;
    float rightWheelpodAngleSetpoint;
    float leftWheelpodAngle;
    float rightWheelpodAngle;
    int turnMotorEffort;
    bool isAngled;
    float driveSpeed;
    long previousOdomReadTime;
    float poseStepX;
    float poseStepY;
    float poseStepTheta;
    float wheelDisplacement[4];
    double cosThetaL;
    double sinThetaL;
    double cosThetaR;
    double sinThetaR;
    float newPostion0[2];
    float newPostion1[2];
    float newPostion2[2];
    float newPostion3[2];
    double angleFromWheel0;
    double angleFromWheel1;
    double angleFromWheel2;
    double angleFromWheel3;
    int state;
    float yICC;
    float left_turn_motor_error;
    float right_turn_motor_error;
    float leftTurnI;
    float rightTurnI;
};