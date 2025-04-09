#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "./msg/EncoderRead.msg"
#include "./msg/EncoderRequest.msg"
#include "./encoder_talk.hpp"

using namespace std::chrono_literals;

/*
Motor communicator is the primary class that runs on the Raspberry Pi and handles all motor communication.
It should have at minimum 2 publishers, one for encoder data and one for motor data (IE what the motors are currently set to after the PID)
It should also have 3 subscriptions, one for drivetrain motor commands, one for excavator commands, and one for deposit commands
*/

/*
Two different modes for encoder communication
CAN BUS for drive train encoders, one collection vertical, one collection run, one deposit
PWM for two turning motors 
*/

class motorCommunicator : public rclcpp::Node{
    public:
        motorCommunicator() 
        : Node("motor_communicator"), count_(0)
        {
            this->create_publisher<motor_comm::msg::EncoderRead>("/mooncake/encoders", 10); //publisher that will publish either all encoder counts or just one
            this->create_wall_timer(100ms, std::bind(&motorCommunicator::turn_encoder_timer_readout, this)); 
            this->create_subscription<motor_comm::msg::EncoderRequest>("/mooncake/encoder_request", 10, call_to_read); //this is where the topic for the encoders was named
            
        }
    private:
        EncoderReader encoders;
        
        void turn_encoder_timer_readout(){
            //read in turning encoders and publish them
            
        }
        void call_to_read(){
            //via a ROS2 topic, 
            //message contains the request of the encoder locaiton so read it and send it
            
        }
};

int main(int argc, char** argv){

}