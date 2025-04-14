#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "./encoder_talk.hpp"
#include <encoder_read__struct.hpp>
#include "motor_comm/include/motor_comm/motor_comm/msg/encoder_request.hpp"
#include "motor_comm/msg/encoder_read.hpp"
#include "motor_comm/msg/turn_read.hpp"
#include "motor_comm/msg/speed_return.hpp"
#include "motor_comm/msg/read_all_encoders_request.hpp"
#include "motor_comm/msg/motor_request.hpp"
#include "./motor_talk.hpp"


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
            readPublisher = this->create_publisher<motor_comm::msg::EncoderRead>("/mooncake/encoders", 10); //publisher that will publish either all encoder counts or just one
            timer_ = this->create_wall_timer(100ms, std::bind(&motorCommunicator::turn_encoder_timer_readout, this)); 
            encoderRequestSub = this->create_subscription<motor_comm::msg::EncoderRequest>("/mooncake/encoder_request", 10, std::bind(&motorCommunicator::call_to_read, this)); //this is where the topic for the encoders was named
            turnPub = this->create_publisher<motor_comm::msg::TurnRead>("/mooncake/turn_readout", 10);
            motorRequestSub = this->create_subscription<motor_comm::msg::MotorRequest>("/mooncake/motor_request", 10, std::bind(&motorCommunicator::set_motor, this));

        }
    private:
    
    void turn_encoder_timer_readout(){
        //read in turning encoders and publish them
        motor_comm::msg::TurnRead msg;
        msg.left = encoders.getAngle(encoders.TURN_LEFT);
        msg.right = encoders.getAngle(encoders.TURN_RIGHT);
        turnPub->publish(msg);
    }
    void call_to_read(motor_comm::msg::EncoderRequest &msg){
        //via a ROS2 topic,  
        //message contains the request of the encoder locaiton so read it and send it
        float readAngle = encoders.getAngle(encoders.convertNumToEn(msg.encoder));
        if(readAngle != -1){
            //if we get an acutal reading
            motor_comm::msg::EncoderRead newMsg; //create a new message
            newMsg.encoder_location = msg.encoder; //add in the encoders location
            newMsg.angle = readAngle; //add the angle read in from the encodeer
            readPublisher->publish(newMsg); //publish the message
        }else{
            return;
        }
    }
    void set_motor(motor_comm::msg::MotorRequest &msg){
        if(msg.has_turn){
            motors.setSpeed(motors.LEFT_TURN, msg.left_wheel_pod_turn);
            motors.setSpeed(motors.RIGHT_TURN, msg.right_wheel_pod_turn);
        }
        if(msg.has_intake_run){

        }
        if(msg.has_intake_vertical){
            motors.setSpeed(motors.INTAKE_VERTICAL, msg.intake_vertical_effort);
        }
        if(msg.has_turn || msg.has_intake_run || msg.has_intake_vertical){ //if we had to change any of the pwm motors
            motors.update(); //run PID
        }
    }
        MotorController motors;
        EncoderReader encoders;
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<motor_comm::msg::EncoderRead>::SharedPtr readPublisher;
        rclcpp::Subscription<motor_comm::msg::EncoderRequest>::SharedPtr encoderRequestSub;
        rclcpp::Publisher<motor_comm::msg::TurnRead>::SharedPtr turnPub;
        rclcpp::Subscription<motor_comm::msg::MotorRequest>::SharedPtr motorRequestSub;
};

int main(int argc, char** argv){
    motorCommunicator motorComm = motorCommunicator();

}