#include "./statemachine.hpp"
//main file of the drivetrain_controller package

using namespace std::chrono_literals;

class robotController: public rclcpp::Node{
    public:
        /*
        the robotController Node has 4 purposes, the first is to publish motor commands to the motor_comm node
        The next is to keep track of the wheels odometry using the encoder sub which gets its info from motor_comm as well
        the third is to get commands from the drive_command topic which the jetson_comm node publishes to
        the last is to update the state machine which keeps track of what state the drive train is in. this is the main 'brain' of the drivetrain.
        */
        robotController() : Node("robot_controller"), count_(0){
            motorPub_ = this->create_publisher<motor_comm::msg::MotorRequest>("/mooncake/driveMotor", 10);
            encoderSub_ = this->create_subscription<motor_comm::msg::EncoderRead>("/mooncake/encoders", 10, std::bind(encoder_read_ready));
            jetsonCommandSub_ = this->create_subscription<drivetrain_controller::msg::JetsonDrivetrainCommand>("/mooncake/driveCommand", 10, std::bind(command_callback));
            loopTimer_ = this->create_wall_timer(100ms, std::bind(loopCallback, this));
        };

    private:
        bool waitingOnEncoder = false;
        bool waitingOnCommand = true;
        void encoder_read_ready(){
            //TODO

        }

        void command_callback(const drivetrain_controller::msg::JetsonDrivetrainCommand &msg) const{
            //jetson_comm handles the twist and angular velocity to JetDrivetrainCommand conversion.
            //math for it is https://github.com/thesamrooney/luna_control/blob/master/src/LunaController.cpp LINE 351 ish
            auto newState = table.find(msg.to_state);
            if(newState != table.end()){
                prevState = currState;
                currState = newState->second;
            }else{ //error occured during the find in the map
                currState = DISABLED;  //disable robot
            }
            
            if(currState == DISABLED){
                auto msg = motor_comm::msg::MotorRequest(); //create the message (it defaults to false and 0s)
                //so we set all the has' to true in order to have the motor communicator read in the values
                msg.oprn_deposit = false;
                msg.has_drive = true;
                msg.has_intake_run = true;
                msg.has_intake_vertical = true;
                msg.has_turn = true;
                motorPub_->publish(msg); //publish the created message
                return; //exit early to disable the robot
            }

            auto message = motor_comm::msg::MotorRequest();
            if(msg.has_turn){ //if the given jetson command has a turn value
                
            }

            if(msg.has_drive){

                auto motor_msg = motor_comm::msg::MotorRequest(); //turn this into the motor_comm::msg::MotorRequest.msg
            }
        }

        void loopCallback(){
            switch(currState){
                case DISABLED:
                break;
                case DRIVE:
                break;
                case POINT_TURN:
                break;
                case ICC_TURN:
                break;
                case LEFT_WHEEL_RECOVERY:
                break;
                case RIGHT_WHEEL_RECOVERY:
                break;
                default: //if we ever end up here something has gone terribly wrong and we should disable the robot to be safe
                    currState = DISABLED; //set current state to DISABLED
                    loopCallback(); //call this function again to actually disable it.
                break;
            }
        }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<motor_comm::msg::MotorRequest>::SharedPtr motorPub_;
    rclcpp::Subscription<motor_comm::msg::EncoderRead>::SharedPtr encoderSub_;
    rclcpp::Subscription<drivetrain_controller::msg::JetsonDrivetrainCommand>::SharedPtr jetsonCommandSub_;
    rclcpp::TimerBase::SharedPtr loopTimer_;
    size_t count_;
       
};
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robotController>());
    rclcpp::shutdown();
    return 0; //generic return
}