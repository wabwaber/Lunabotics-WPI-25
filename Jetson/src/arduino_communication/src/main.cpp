#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

/*
TODO:
get drive motor top speed
understand what the drive motor controllers need (c620)
understand what the 0 angle is (wheels parallel to the white tube)
understand how the turn motors are controlled (talon)
*/

class MinimalPublisher : public rclcpp::Node {
    public:
        MinimalPublisher() 
        : Node("minimal_publisher"), count_(0)
        {
            auto publisher_ = this->create_publisher<nav_msgs::msg::Path>("", 10);
            auto timer_ = this->create_wall_timer(
                500ms, std::bind(&MinimalPublisher::timer_callback, this));
        }
    private:
        void timer_callback(){
            auto message = nav_msgs
        }
}

int main(int argc, char** argv){
    


    return 0; //default exit
}