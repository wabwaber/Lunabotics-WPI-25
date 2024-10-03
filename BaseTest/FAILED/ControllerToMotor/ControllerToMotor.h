#include "AA_MCP2515.h"
class ControllerToMotor{
    public:
        bool setMotors(int LF, int LB, int RB, int RF);
        bool setMotors(int L, int R);
        bool setMotors(int A);
    private:
        int DriveDLC = 8; //CAN message size for the drivetrain
        int ConveyorDLC = 2; //CAN message size for the conveyor system (probably intake)
        int CANIDDrive = 0x200; //512 Drivetrain CAN ID
        int CANIDConveyor = 0x1FF; //511 Conveyor CAN ID
        int ConversionFactor = 256; //used for CAN Communication abstraction
        const int CAN_CS = 42; //Chip select pin (to MP2515)
        const int CAN_SI = 37; //SPI Data input pin (to MP2515)
        const int CAN_SO = 39; //SPI Data output pin (to MP2515)
        const int CAN_SCK = 35; //SPI Clock input (to MP2515)
/*NOTWIRED*/        const int CAN_INT = 2; //Interrupt pin (to MP2515)

};