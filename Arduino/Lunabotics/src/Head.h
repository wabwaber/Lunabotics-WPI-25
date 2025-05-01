#include <Arduino.h>
#include <ArdyToMot.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>


struct steer_command{
    time_t timestamp;
    float angle_L; //angle of the front left wheel (degrees)
    float angle_R; //angle of hte front right wheel
};

struct motion_command{
    time_t timestamp;
    float DTT_FL; //stands for distance to turn _ front left ; referring to the distance the front left wheel must travel (meters)
    float DTT_FR;
    float DTT_BL;
    float DTT_BR;
};