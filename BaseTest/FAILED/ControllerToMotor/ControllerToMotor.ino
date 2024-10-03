#include <AA_MCP2515.h>
//Constants
const CANBitrate::Config CAN_BITRATE = CANBitrate::Config_8MHz_1000kbps;
const uint8_t CAN_CS = 41; //Chip select pin (to MP2515)
const uint8_t CAN_SI = 37; //SPI Data input pin (to MP2515)
const uint8_t CAN_SO = 39; //SPI Data output pin (to MP2515)
const uint8_t CAN_SCK = 35; //SPI Clock input (to MP2515) 
const int8_t CAN_INT = 2; //Interrupt pin (to MP2515)                             NOT WIRED
//we have a Arduino Mega 2560 Rev 3
  //runs at 16mhz
  //single core :(
//Its possible to cancel a transmission if ABAT bit is set, idk what to but my guess would be 1.
//The bit MUST be reset before transmiting can continue, (reset it after verifying that the TXREQ (buffer) have been cleared)
//4 brushless motor controllers
//2 brushed motor controllers
CANConfig config(CAN_BITRATE, CAN_CS, CAN_INT);
CANController CAN(config);
int CANIDDrive = 0x200; //512 Drivetrain system ID
int CANIDConveyor = 0x1FF; //511 Conveyor system ID
int DriveDLC = 8; //CAN message size (DLC)
int ConveyorDLC = 2; //CAN message size (DLC)
int ConversionFactor = 256; //used for the drive message, in / % / % pattern for motors 0-3 going 0 0 1 1 2 2 3 3.
uint8_t data[8];//data is 8 bytes of data for the frame

//Function Prototypes
bool setMotors(int LF, int LB, int RB, int RF);
bool setMotors(int L, int R);
bool setMotors(int A);

//setup function to initialize anything
void setup() {
  // put your setup code here, to run once:
  
}

void init(){
  Serial.begin(115200);
  while(CAN.begin(CANController::Mode::Normal) != CANController::OK){ //try to initate the controller if its not ok then
    Serial.println("CAN no worky :("); //inform user
    delay(1000); //wait 1 second
  }
  //once its working
  Serial.println("CAN worky :)"); //inform user
}

//setAll sets all the motors to the speed A. It returns a bool saying if it was successful in doing so
bool setMotors(int A){
  float packet[] = {A / ConversionFactor, A % ConversionFactor, A / ConversionFactor, A % ConversionFactor, A / ConversionFactor, A % ConversionFactor, A / ConversionFactor, A % ConversionFactor};
  CANController::IOResult result = CAN.write(CANIDDrive, packet, sizeof(packet));
  return result == CANController::IOResult::OK;
}

bool setMotors(int L, int R){
  float packet[] = {L / ConversionFactor, L % ConversionFactor, L / ConversionFactor, L % ConversionFactor, R / ConversionFactor, R % ConversionFactor, R / ConversionFactor, R % ConversionFactor};
  CANController::IOResult result = CAN.write(CANIDDrive, packet, sizeof(packet));
  return result == CANController::IOResult::OK;
}
bool setMotors(int LF, int LB, int RB, int RF){
  float packet[] = {LF / ConversionFactor, LF % ConversionFactor, LB / ConversionFactor, LB % ConversionFactor, RB / ConversionFactor, RB % ConversionFactor, RF / ConversionFactor, RF % ConversionFactor};
  CANController::IOResult result = CAN.write(CANIDDrive, packet, sizeof(packet));
  return result == CANController::IOResult::OK;
}
