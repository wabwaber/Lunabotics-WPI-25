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
            canSub_ = this->create_subscription<motor_comm::msg::SpeedReturn>("/mooncake/motor_speed", 10, std::bind(update_speeds_callback));
            encoderSub_ = this->create_subscription<motor_comm::msg::EncoderRead>("/mooncake/encoders", 10, std::bind(encoder_read_ready));
            jetsonCommandSub_ = this->create_subscription<drivetrain_controller::msg::JetsonDrivetrainCommand>("/mooncake/driveCommand", 10, std::bind(command_callback));
            loopTimer_ = this->create_wall_timer(100ms, std::bind(loopCallback, this));
            currState = DISABLED;
            canMotorSpeeds[0] = 0;
            canMotorSpeeds[1] = 0;
            canMotorSpeeds[2] = 0;
            canMotorSpeeds[3] = 0;
            left_turn_setpoint = 0;
            right_turn_setpoint = 0;
            left_turn_angle = 0;
            right_turn_angle = 0;
            turn_motor_effort = 0;
            target_drive_speeds[0] = 0;
            target_drive_speeds[1] = 0;
            target_drive_speeds[2] = 0;
            target_drive_speeds[3] = 0;
            pose_step_x = 0;
            pose_step_y = 0;
            pose_step_theta = 0;
            shouldIntakeBeDown = false;
            shouldIntakeBeRunning = false;
        };

    private:
        bool waitingOnEncoder = false;
        bool waitingOnCommand = true;
        void encoder_read_ready(){

            //TODO
            //we have one encoder reading in so we should calculate the speed that it is running at
            
        }

        void update_speeds_callback(motor_comm::msg::SpeedReturn &msg){
            canMotorSpeeds[0] = msg.fl_drive;
            canMotorSpeeds[1] = msg.bl_drive;
            canMotorSpeeds[2] = msg.br_drive;
            canMotorSpeeds[3] = msg.fr_drive;
        }

        void command_callback(const drivetrain_controller::msg::JetsonDrivetrainCommand &msg) const{
            //jetson_comm handles the twist and angular velocity to JetDrivetrainCommand conversion.
            //math for it is https://github.com/thesamrooney/luna_control/blob/master/src/LunaController.cpp LINE 351 ish
            auto newState = table.find(msg.to_state);
            if(newState != table.end()){
                prevState = currState;
                currState = newState->second;
                if(currState != DISABLED){
                    //skip if we are disabling the robot
                    //also setting the turn points here as the currState will control which ones are actually used when the loop runs
                    if(msg.has_turn){
                        right_turn_setpoint = msg.right_wheel_pod_turn;
                        left_turn_setpoint = msg.left_wheel_pod_turn;
                    }
                    if(msg.has_drive){
                        target_drive_speeds[0] = msg.fl_drive;
                        target_drive_speeds[1] = msg.bl_drive;
                        target_drive_speeds[2] = msg.br_drive;
                        target_drive_speeds[3] = msg.fr_drive;
                    }
                    if(msg.has_intake_run){
                        
                    }
                    if(msg.has_intake_vertical){
                        
                    }
                }

            }else{ //error occured during the find in the map
                currState = DISABLED;  //disable robot
            }
            
            if(currState == DISABLED){
                auto msg = motor_comm::msg::MotorRequest(); //create the message (it defaults to false and 0s)
                //so we set all the has' to true in order to have the motor communicator read in the values
                msg.open_deposit = false;
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
            
            motor_comm::msg::MotorRequest mesg;

            switch(currState){
                case DISABLED:
                    mesg.bl_drive = 0;
                    mesg.br_drive = 0;
                    mesg.fl_drive = 0;
                    mesg.fr_drive = 0;
                    mesg.intake_run = 0;
                    mesg.intake_vertical_effort = 0;
                    mesg.has_drive = true;
                    mesg.has_intake_run = true;
                    mesg.has_intake_vertical = true;
                    mesg.left_wheel_pod_turn = -1;
                    mesg.right_wheel_pod_turn = -1;
                    motorPub_->publish(mesg);
                break;
                case DRIVE: //drive straight
                    left_turn_setpoint = 0.0;
                    right_turn_setpoint = 0.0;
                    mesg.left_wheel_pod_turn = left_turn_setpoint;
                    mesg.right_wheel_pod_turn = right_turn_setpoint;
                    mesg.fl_drive = canMotorSpeeds[0];
                    mesg.bl_drive = canMotorSpeeds[1];
                    mesg.br_drive = canMotorSpeeds[2];
                    mesg.fr_drive = canMotorSpeeds[3];
                    mesg.has_drive = true;
                    mesg.has_turn = true;
                    motorPub_->publish(mesg);
                break;
                case POINT_TURN:
                    left_turn_setpoint = M_PI_2;
                    right_turn_setpoint = -M_PI_2;

                break;
                case ICC_TURN:

                break;
                case LEFT_WHEEL_RECOVERY:
                break;
                case RIGHT_WHEEL_RECOVERY:
                break;
                case RECOVERY:
                break;
                case AUTO_COLLECTION:
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
    rclcpp::Publisher<motor_comm::msg::MotorRequest>::SharedPtr canPub_;
    rclcpp::Subscription<motor_comm::msg::SpeedReturn>::SharedPtr canSub_;
       
};
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<robotController>());
    rclcpp::shutdown();
    return 0; //generic return
}