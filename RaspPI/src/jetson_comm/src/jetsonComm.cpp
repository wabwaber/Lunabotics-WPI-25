#include "./jetsonComm.hpp"

//The code within twist_handler have this copyright disclaimer
// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

/*
 * This code has been adapted and/or modified for use in the 2024-25 WPI Lunabotics MQP.
 * Author: Sam Rooney & Matthew Copeland
 */

void jetsonCommunicator::twist_handler(geometry_msgs::TwistStamped &msg){
    std::shared_ptr<geometry_msgs::Twist> last_command_msg;
    geometry_msgs::TwistStamped command = msg;
    auto &linear_command = command.twist.linear.x;
    auto &strafe_command = command.twist.linear.y;
    auto &angular_command = command.twist.angular.z;
    if(lastLoop != -1){
        period = (time_t)time - lastLoop;
    }

    
    double LF_feedback_mean = 0.0;
    double LB_feedback_mean = 0.0;
    double RB_feedback_mean = 0.0;
    double RF_feedback_mean = 0.0;
    double LF_turn_feedback_mean = 0.0;
    double LB_turn_feedback_mean = 0.0;
    double RB_turn_feedback_mean = 0.0;
    double RF_turn_feedback_mean = 0.0;
    
    for(size_t i = 0; i < NUM_OF_WHEELS_PER_SIDE; i++){
        const double LF_feedback = registered_LF_handles[i].feedback.get().get_value();
        const double LB_feedback = registered_LB_handles[i].feedback.get().get_value();
        const double RB_feedback = registered_RB_handles[i].feedback.get().get_value();
        const double RF_feedback = registered_RF_handles[i].feedback.get().get_value();

        const double LF_turn_feedback = registered_LF_turn_handles[i].feedback.get().get_value();
        const double LB_turn_feedback = registered_LB_turn_handles[i].feedback.get().get_value();
        const double RB_turn_feedback = registered_RB_turn_handles[i].feedback.get().get_value();
        const double RF_turn_feedback = registered_RF_turn_handles[i].feedback.get().get_value();
        

        if(std::isnan(LF_feedback) || std::isnan(LB_feedback) || std::isnan(RB_feedback) || std::isnan(RF_feedback)){
            RCLCPP_ERROR(this->get_logger(), "One of the drive wheels failed to init!");
            return;
        }

        if(std::isnan(LF_turn_feedback) || std::isnan(LB_turn_feedback) || std::isnan(RB_turn_feedback) || std::isnan(RF_turn_feedback)){
            RCLCPP_ERROR(this->get_logger(), "either the left or right wheel turn pods failed to init");
            return;
        }

        LF_feedback_mean += LF_feedback;
        LB_feedback_mean += LB_feedback;
        RB_feedback_mean += RB_feedback;
        RF_feedback_mean += RF_feedback;

        LF_turn_feedback_mean += LF_turn_feedback;
        LB_turn_feedback_mean += LB_turn_feedback;
        RB_turn_feedback_mean += RB_turn_feedback;
        RF_turn_feedback_mean += RF_turn_feedback;
    }
        LF_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);
        LB_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);
        RB_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);
        RF_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);

        LF_turn_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);
        LB_turn_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);
        RB_turn_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);
        RF_turn_feedback_mean /= static_cast<double>(NUM_OF_WHEELS_PER_SIDE);

        auto &last_command = previous_commands.back.twist;
        auto &second_to_last_command = previous_commands.front.twist;
        limiter_linear_.limit(linear_command, last_command.linear.x, second_to_last_command.linear.x, period.seconds());
        
        double LF_velocity = 0.0;
        double LB_velocity = 0.0;
        double RB_velocity = 0.0;
        double RF_velocity = 0.0;

        double LF_turn_position = 0.0;
        double LB_turn_position = 0.0;
        double RB_turn_position = 0.0;
        double RF_turn_position = 0.0;
    
        if(angular_command == 0.0 && linear_command == 0.0 && strafe_command == 0.0){
            LF_velocity = -strafe_command / WHEEL_RADIUS;
            LB_velocity = strafe_command / WHEEL_RADIUS;
            RB_velocity = -strafe_command / WHEEL_RADIUS;
            RF_velocity = strafe_command / WHEEL_RADIUS;

            LF_turn_position = -M_PI_2;
            LB_turn_position = M_PI_2;
            RB_turn_position = -M_PI_2;
            RF_turn_position = M_PI_2;
        }
        else if(angular_command == 0.0){
            LF_velocity = linear_command / WHEEL_RADIUS;
            LB_velocity = linear_command / WHEEL_RADIUS;
            RB_velocity = linear_command / WHEEL_RADIUS;
            RF_velocity = linear_command / WHEEL_RADIUS;
            LF_turn_position = 0.0;
            LB_turn_position = 0.0;
            RB_turn_position = 0.0;
            RF_turn_position = 0.0;
        }
        else if(linear_command == 0.0){
            const double theta_L = (M_PI / 2.0) - atan(WHEEL_TRACK / WHEEL_BASE);
            const double theta_R = -((M_PI / 2.0) - atan(WHEEL_TRACK / WHEEL_BASE));
            const double R = sqrt(pow(WHEEL_TRACK / 2, 2) + pow(WHEEL_BASE / 2, 2));
            const double v = angular_command * R / WHEEL_RADIUS;

            LF_velocity = -v;
            LB_velocity = -v;
            RB_velocity = v;
            RF_velocity = v;
            LF_turn_position = -theta_L;
            LB_turn_position = theta_L;
            RB_turn_position = theta_R;
            RF_turn_position = -theta_R;
        }
        else{
            const double icc = linear_command/angular_command;
            const double theta_L = -atan((WHEEL_BASE/2) / (icc - (WHEEL_TRACK / 2)));
            const double theta_R = -atan((WHEEL_BASE / 2) / (icc + (WHEEL_TRACK / 2)));
            const double R_L = pow(pow(WHEEL_BASE / 2, 2) + pow(icc - (WHEEL_TRACK / 2), 2), 0.5);
            const double R_R = pow(pow(WHEEL_BASE / 2, 2) + pow(icc + (WHEEL_TRACK / 2), 2), 0.5);

            const double v_L = linear_command * abs(R_L / icc) / WHEEL_RADIUS;
            const double v_R = linear_command * abs(R_R / icc) / WHEEL_RADIUS;

            LF_velocity = v_L;
            LB_velocity = v_L;
            RB_velocity = v_R;
            RF_velocity = v_R;

            LF_turn_position = -theta_L;
            LB_turn_position = theta_L;
            RB_turn_position = theta_R;
            RF_turn_position = -theta_R;
        }
        const double LF_turn_delta = abs(LF_turn_position - LF_turn_feedback_mean);
        const double LB_turn_delta = abs(LB_turn_position - LB_turn_feedback_mean);
        const double RB_turn_delta = abs(RB_turn_position - RB_turn_feedback_mean);
        const double RF_turn_delta = abs(RF_turn_position - RB_turn_feedback_mean);
        const bool allow_wheel_movement = (
            LF_turn_delta < ALLOWED_TURN_DRIVE_ANGLE &&
            LB_turn_delta < ALLOWED_TURN_DRIVE_ANGLE &&
            RB_turn_delta < ALLOWED_TURN_DRIVE_ANGLE &&
            RF_turn_delta < ALLOWED_TURN_DRIVE_ANGLE
        );

        for (size_t i = 0; i < static_cast<size_t>(NUM_OF_WHEELS_PER_SIDE); i++){
            if(allow_wheel_movement){
                registered_LF_handles[i].velocity.get().set_value(LF_velocity);
                registered_LB_handles[i].velocity.get().set_value(LB_velocity);
                registered_RB_handles[i].velocity.get().set_value(RB_velocity);
                registered_RF_handles[i].velocity.get().set_value(RF_velocity);
            }
            else{
                registered_LF_handles[i].velocity.get().set_value(0.0);
                registered_LB_handles[i].velocity.get().set_value(0.0);
                registered_RB_handles[i].velocity.get().set_value(0.0);
                registered_RF_handles[i].velocity.get().set_value(0.0);
            }
            registered_LF_turn_handles[i].position.get().set_value(LF_turn_position);
            registered_LB_turn_handles[i].position.get().set_value(LB_turn_position);
            registered_RB_turn_handles[i].position.get().set_value(RB_turn_position);
            registered_RF_turn_handles[i].position.get().set_value(RF_turn_position);
            return; //done
        }
}
