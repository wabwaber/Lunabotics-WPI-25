#pragma once //run this header once
//the header file for robot_controller but called differently as it contains constants and structs to use in .cpp file
#include "rclcpp.hpp"
#include <string.h>
#include <chrono>
#include <functional>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "motor_comm/msg/encoder_read.hpp"
#include "drivetrain_controller/msg/jetson_drivetrain_command.hpp"
#include "motor_comm/msg/motor_request.hpp"
#include "motor_comm/msg/speed_return.hpp"


//TODO change these
#define WHEEL_TRACK 0.0
#define WHEEL_BASE 0.0
#define WHEEL_RADIUS 0.0
#define NUM_OF_WHEELS_PER_SIDE 2

float ALLOWED_TURN_DRIVE_ANGLE = 3.1416;

//states taken from last years code
enum state {
    DISABLED = 0, //drivetrain doesnt do anything
    DRIVE = 1, //Drive straight/dont move turn motors during this state
    POINT_TURN = 2, //rotate wheels and move all drive motors at same speed
    ICC_TURN = 3, //arc/spline movement, ICC stands for instaneous center of curvature
    LEFT_WHEEL_RECOVERY = 4,
    RIGHT_WHEEL_RECOVERY = 5,
    RECOVERY = 6
};


//ultimatum@skibidy.dyn.wpi.edu Unfunded-Clover4-Stinging
state currState;
state prevState;

//okay this is here so that I can quickly convert the string in the message to the enum. without having if else or having the message be large in size because keep in mind it will be sent every time the Jetson updates the robots position 
static std::unordered_map<std::string, state> const table = {
    {"DISABLED",state::DISABLED},
    {"DRIVE",state::DRIVE},
    {"POINT_TURN",state::POINT_TURN},
    {"ICC_TURN", state::LEFT_WHEEL_RECOVERY},
    {"RIGHT_WHEEL_RECOVERY", state::RIGHT_WHEEL_RECOVERY}
};


float left_turn_setpoint;
float right_turn_setpoint;
float left_turn_angle;
float right_turn_angle;
float turn_motor_effort;
float target_drive_speed;
float canMotorSpeeds[4];
float pose_step_x;
float pose_step_y;
float pose_step_theta;