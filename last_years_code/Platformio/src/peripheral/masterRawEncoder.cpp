#include "masterRawEncoder.h"
#include <I2C.h>

RawMasterEncoder::RawMasterEncoder() {}

void RawMasterEncoder::init(int id, int multiplexerId) {
    encoderNumber = id;
    multiplexerNumber = multiplexerId;
}

int RawMasterEncoder::getRawAngle() {

    if (multiplexerNumber == 0) {
        I2c.write(0x70, 0x00, 1 << encoderNumber);
        I2c.write(0x71, 0x00, 1 << 7);
    } else {
        I2c.write(0x71, 0x00, 1 << encoderNumber);
        I2c.write(0x70, 0x00, 1 << 7);
    }

    // Encoder reading
    I2c.read(0x36, 0x0C, 2); // Request bytes
    highbyte = I2c.receive() & 0x0F;
    
    lowbyte = I2c.receive();

    if(lowbyte != 0 || highbyte != 0){ // if we have a viable read
        out = (highbyte << 8) | lowbyte; // Combine bytes into out
    }

    return out;
}
