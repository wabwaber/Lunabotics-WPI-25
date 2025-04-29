
#include "rclcpp/rclcpp.hpp"
#include "motor_comm/msg/motor_request.hpp"
#include "geometry_msgs/TwistStamped.h"
#include "drivetrain_controller/msg/jetson_drivetrain_command.hpp"
#include "geometry_msgs/Twist.h"
#include "speed_limiter.hpp"
#include "statemachine.hpp"
#include "controller_interface.hpp"

class jetsonCommunicator : public rclcpp::Node{
    public:
        jetsonCommunicator() : Node("jetson_communicator"), count_(0){
            drivetrain_request_pub = this->create_publisher<drivetrain_controller::msg::JetsonDrivetrainCommand>("mooncake/motor_request", 10); 
            jetson_twist_sub = this->create_subscription<geometry_msgs::TwistStamped>("mooncake/twists", 10, std::bind(&jetsonCommunicator::twist_handler, this));
        };
    
    private:
        size_t count_;
        rclcpp::Publisher<drivetrain_controller::msg::JetsonDrivetrainCommand>::SharedPtr drivetrain_request_pub;
        time_t age_of_last_command;
        void twist_handler(geometry_msgs::TwistStamped &msg);
        rclcpp::Subscription<geometry_msgs::TwistStamped>::SharedPtr jetson_twist_sub;
    //things below are taken from the code on Sam's github the same is also probably true for their implementations
    protected:
        struct WheelHandle{
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity;
        };
        struct PodHandle{
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> feedback;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> position;
        };

        //the following is extremely cursed but it will allow us to have a 'queue' of StampedTwists. As the std::queue does not support Twists
        struct queue{
            geometry_msgs::TwistStamped front;
            geometry_msgs::TwistStamped back;
        };

        bool reset();
        void halt();
        bool is_halted();
        time_t lastLoop = -1;
        luna_controller::SpeedLimiter limiter_linear_;
        luna_controller::SpeedLimiter limiter_strafe_;
        luna_controller::SpeedLimiter limiter_angular_;
        std::vector<WheelHandle> registered_LF_handles;
        std::vector<WheelHandle> registered_LB_handles;
        std::vector<WheelHandle> registered_RB_handles;
        std::vector<WheelHandle> registered_RF_handles;

        std::vector<PodHandle> registered_LF_turn_handles;
        std::vector<PodHandle> registered_LB_turn_handles;
        std::vector<PodHandle> registered_RB_turn_handles;
        std::vector<PodHandle> registered_RF_turn_handles;

        jetsonCommunicator::queue previous_commands;
};