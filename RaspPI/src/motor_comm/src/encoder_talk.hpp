#include <stdlib.h>
#include <cstddef>
#include <pi_i2c.h>
#include <iostream>


#define BYTES_2_RAD 0.001533981

#define I2C_BUS_Count 6

//below are all the I2C buses for a raspberry PI. the list of all the pins and their associated I2C buses can be seen below
#define I2C_DATA_BUS_0 0
#define I2C_CLOCK_BUS_0 1

#define I2C_DATA_BUS_1 2
#define I2C_CLOCK_BUS_1 3

#define I2C_DATA_BUS_2 4
#define I2C_CLOCK_BUS_2 5

#define I2C_DATA_BUS_3 6
#define I2C_CLOCK_BUS_3 7

#define I2C_DATA_BUS_4 10
#define I2C_CLOCK_BUS_4 11

#define I2C_DATA_BUS_5 22
#define I2C_CLOCK_BUS_5 23

//these are all subject to change depending on how sam ends up wiring the robot
enum encoder {INTAKE_ROTATE=0, INTAKE_VERTICAL=1, TURN_LEFT=2, TURN_RIGHT=3, DEPOSIT=4};

 class EncoderReader{
    public:
        EncoderReader();
        int getAngle(encoder get);
    private:
        bool initalizeI2C();
        float* currAngle;
        float* prevAngle;
        int* rawAngle;
        int lastReturnValue;
        //okay this is interesting as when it scans it will add a 1 to the addresses (index of array) if there is a device at that indexes (addresses) location
        int* addressBook = (int*) calloc(127, sizeof(int*));
        int** I2CBusses = (int**) calloc(6, sizeof(int*));
};

/*
source: https://elinux.org/RPi_BCM2711_GPIOs#SDA0
for the I2C above there are 5 total I2C master data/clock lines on a Raspberry PI 4
i2c0 data: GPIO 0, GPIO 28, GPIO 44, GPIO 46
i2c0 clock: GPIO 1, GPIO 29, GPIO 45, GPIO 47

i2c1 data: GPIO2, GPIO 44, GPIO 46
i2c1 clock: GPIO3, GPIO 45, GPIO 47

i2c3 data: GPIO 2, GPIO 4
i2c3 clock: GPIO 3, GPIO 5

i2c4 data: GPIO 6, GPIO 8
i2c4 clock: GPIO 7, GPIO 9

i2c5 data: GPIO 10, GPIO 12
i2c5 clock: GPIO 11, GPIO 13

i2c6 data: GPIO 0, GPIO 22
i2c6 cock: GPIO 1, GPIO 23
*/