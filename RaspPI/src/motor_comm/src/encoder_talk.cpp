#include "encoder_talk.hpp"
/*
This initalizes the required variables and functions in order to get encoder readings
counts is a 2D array with each location in the first array containing a size 2 array containing the number of encoders and the multiplexer ID (in that order)
*/
EncoderReader::EncoderReader(){
    this->initalizeI2C();
    this->startAngle = (float*)malloc(sizeof(float)*6);
    this->lastReturnValue = 0;
}

/*
Gets the encoder angle reading based on the given enum, see .hpp for valid values
*/
int EncoderReader::getAngle(encoder encoderToGet){
    float angle = -1.0;

    //the writing part of last years code was for the multiplexers (ignore this if you don't know what it means because it doesn't apply anymore)
 
    //reconfigure the I2C to use the correct set of data and clock pins
    config_i2c(this->I2CBusses[encoderToGet][1], this->I2CBusses[encoderToGet][0], I2C_STANDARD_MODE);

    //read from the encoders
    int data[2]; //16 bytes 2 slots in array 1 int being 8 bytes (allegedly) and we need 12 to 2 slots
    read_i2c(0x36, 0x0C, data, 2); //data[0] is MSB and data[1] is LSB (_ significant byte)
    int out = ((data[0] & 0x0F) << 8) | data[1];

    angle = static_cast<float>(out) * BYTES_2_RAD;
    this->prevAngle[encoderToGet] = this->currAngle[encoderToGet];
    this->currAngle[encoderToGet] = angle;

    return angle;
}

/*
Initializes all I2C devices across all multiplexers
returns true if this process was successful and false otherwise
*/
bool EncoderReader::initalizeI2C(){
    bool pass = true;
    int bus0[2] = {I2C_CLOCK_BUS_0, I2C_DATA_BUS_0};
    int bus1[2] = {I2C_CLOCK_BUS_1, I2C_DATA_BUS_1};
    int bus2[2] = {I2C_CLOCK_BUS_2, I2C_DATA_BUS_2};
    int bus3[2] = {I2C_CLOCK_BUS_3, I2C_DATA_BUS_3};
    int bus4[2] = {I2C_CLOCK_BUS_4, I2C_DATA_BUS_4};
    int bus5[2] = {I2C_CLOCK_BUS_5, I2C_DATA_BUS_5};
    int* busses[6] = {bus0, bus1, bus2, bus3, bus4, bus5};
    this->I2CBusses = busses; //this is cursed and I hate it TODO: fix this so its not doing this workaround

    for(int i = 0; i < 6; i++){ //put all GPIO pairs into the list while also getting the addresses for each one on 

        config_i2c(this->I2CBusses[i][1], this->I2CBusses[i][0], I2C_STANDARD_MODE);

        if(this->lastReturnValue = scan_bus_i2c(this->addressBook) < 0){
            pass = false;
            std::cout << "error during i2c init: " << this->lastReturnValue << "\n";
        }
        int initData[2];
        read_i2c(0x36, 0x0C, initData, 2); //data[0] is MSB and data[1] is LSB (_ significant byte)
        int out = ((initData[0] & 0x0F) << 8) | initData[1];
        float angle = static_cast<float>(out) * BYTES_2_RAD;
        this->currAngle[i] = angle;
        this->prevAngle[i] = angle;
        this->rawAngle[i] = out;
    }

    return pass;
}