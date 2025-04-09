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


//states taken from last years code
enum state {
    DISABLED = 0, //drive train doesnt do anything
    DRIVE = 1, //Drive straight/dont move turn motors during this state
    POINT_TURN = 2, //rotate wheels and move all drive motors at same speed
    ICC_TURN = 3, //arc/spline movement, ICC stands for instaneous center of curvature
    LEFT_WHEEL_RECOVERY = 4,
    RIGHT_WHEEL_RECOVERY = 5,
};
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