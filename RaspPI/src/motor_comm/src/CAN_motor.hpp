
enum motorLocationDescriptor{DRIVE_FL = 0, DRIVE_FR = 1, DRIVE_BL=2, DRIVE_BR=3, DEPOSIT=6, INTAKE_VERT=7, INTAKE_RUN=8};
#include "motor_talk.hpp"


//https://www.fysetc.com/products/fysetc-ucan-board?_pos=1&_sid=ed350dab0&_ss=r
//CAN Bus controller being used ^
class CANmotor{
    public:
        CANmotor();
        int16 getSpeed(motorLocationDescriptor here);
        
        
    private:
        motorLocationDescriptor loc;
        int CANid; //only applicable if the motor comm type is CAN
        //all CAN pins are in motor_talk.hpp



}