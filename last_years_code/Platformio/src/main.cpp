#include <I2C.h>
#include <Arduino.h>
#include "./robotMap.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "./subsystems/drivetrain.h"
#include "./subsystems/drivetrain.h"
#include "./subsystems/conveyor.h"
#include "./subsystems/deposit.h"
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

std_msgs::String ianOutputMsg;
ros::Publisher ianOutputPub("/listener/ian_output", &ianOutputMsg);

std_msgs::Float32MultiArray poseStep;
ros::Publisher poseStepPub("/jetson/pose_step", &poseStep);

std_msgs::Float32 conveyorSpeed;
ros::Publisher conveyorSpeedPub("/jetson/conveyor_speed", &conveyorSpeed);

std_msgs::Bool plungeTop;
ros::Publisher plungeTopPub("/jetson/plunge_top", &plungeTop);

std_msgs::Bool plungeBot;
ros::Publisher plungeBotPub("/jetson/plunge_bot", &plungeBot);

std_msgs::Bool watchdogBool;
ros::Publisher watchdogBoolPub("/watchdog_bool", &watchdogBool);

Drivetrain drivetrain;
Conveyor conveyor;
Deposit deposit;

int driveSpeed = 0;
bool drivetrainEnable = false;
bool drivetrainAngle = false;

float poseStepVals[3] = { 0.0, 0.0, 0.0};

int odomIterator = 0;

float icc = 0.0;

static unsigned long previousDriveMillis = 0;
static unsigned long previousConveyorMillis = 0;
static unsigned long previousWatchdogMillis = 0;
unsigned long currentMillis = millis();

void drivetrainSpeedCallback(const std_msgs::Float32 &driveSpeedMsg) {
  float driveSpeed = driveSpeedMsg.data;
  drivetrain.setDriveSpeed(driveSpeed);
}

void drivetrainSwitchStateCallback(const std_msgs::Int32 &driveStateMsg) {
  int drivetrainState = driveStateMsg.data;
  drivetrain.setState(drivetrainState);
}


void drivetrainICCallback(const std_msgs::Float32 &driveICCMsg) {
  float icc = driveICCMsg.data;
  drivetrain.setYICC(icc);
}

void conveyorCurrentCallback(const std_msgs::Int32 &conveyorCurrent) {
  conveyor.setConveyorCurrent(conveyorCurrent.data);
}

void conveyorPlungeCallback(const std_msgs::Int32 &plungeSpeed){
  conveyor.setPlungeSpeed(plungeSpeed.data);
}

void depositOpenCallback(const std_msgs::Bool &depositOpenMsg) {
  deposit.setOpen(depositOpenMsg.data);
}

ros::Subscriber<std_msgs::Float32> driveSpeedSub("/drivetrain/drive", &drivetrainSpeedCallback);
ros::Subscriber<std_msgs::Int32> driveStateSub("/drivetrain/state", &drivetrainSwitchStateCallback);
ros::Subscriber<std_msgs::Float32> driveICCSub("/drivetrain/icc", &drivetrainICCallback);
ros::Subscriber<std_msgs::Int32> conveyorSub("/digger/conveyor_current", &conveyorCurrentCallback);
ros::Subscriber<std_msgs::Int32> plungeSub("/digger/plunge", &conveyorPlungeCallback);
ros::Subscriber<std_msgs::Bool> depositOpen("/deposit/open", &depositOpenCallback);

void setup()
{
  Serial.begin(115200);  // Set baud rate to 115200
  nh.getHardware()->setBaud(115200);  // Tell rosserial to use the same baud rate
  nh.initNode();

  nh.advertise(ianOutputPub);
  nh.advertise(poseStepPub);
  nh.advertise(conveyorSpeedPub);
  nh.advertise(plungeTopPub);
  nh.advertise(plungeBotPub);
  nh.advertise(watchdogBoolPub);

  nh.subscribe(driveSpeedSub);
  nh.subscribe(driveStateSub);
  nh.subscribe(driveICCSub);
  nh.subscribe(conveyorSub);
  nh.subscribe(plungeSub);
  nh.subscribe(depositOpen);

  drivetrain.init();
  conveyor.init();
  deposit.init();

  SPI.begin();
  I2c.begin();

  // Set watchdog bool to true
  watchdogBool.data = true;
}

void loop()
{
  currentMillis = millis();

  // Conveyor gets looped every 10 milliseconds
  if (currentMillis - previousConveyorMillis >= CONVEYOR_INTERVAL) {
    previousConveyorMillis = currentMillis;
    
    conveyor.loop();
    
    plungeBot.data = conveyor.isAtBot();
    plungeBotPub.publish(&plungeBot);

    plungeTop.data = conveyor.isAtTop();
    plungeTopPub.publish(&plungeTop);

    conveyorSpeed.data = conveyor.getConveyorSpeed();
    conveyorSpeedPub.publish(&conveyorSpeed);

    deposit.loop();
  }


  // Drivetrain gets looped every X milliseconds
  if (currentMillis - previousDriveMillis >= DRIVETRAIN_INTERVAL) {

    previousDriveMillis = currentMillis;

    nh.spinOnce();

    drivetrain.loop();

    // update Odom
    odomIterator ++;
    if(odomIterator >= ODOM_FREQUENCY){

      drivetrain.stepOdom();

      poseStepVals[0] = drivetrain.getPoseStepX();
      poseStepVals[1] = drivetrain.getPoseStepY();
      poseStepVals[2] = drivetrain.getPoseStepTheta();

      std_msgs::Float32MultiArray stepMsg;
      stepMsg.data_length = 3;
      stepMsg.data = poseStepVals;
      poseStepPub.publish(&stepMsg);

      odomIterator = 0;
    }
  }

  // Arudino Watchdog TImer gets published every quarter second
  if (currentMillis - previousWatchdogMillis >= WATCHDOG_INTERVAL) {
    previousWatchdogMillis = currentMillis;

    watchdogBoolPub.publish(&watchdogBool);
  }

}