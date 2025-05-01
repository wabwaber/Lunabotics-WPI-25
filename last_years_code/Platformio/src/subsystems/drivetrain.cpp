/*
 * drivetrain.cpp
 * 
 *  Created on: Dec 13, 2023
 *      Author: Speeep
 */
#include "drivetrain.h"

Drivetrain::Drivetrain(){}

void Drivetrain::init() {
    left_turn_motor.init(LEFT_TURN_PWM, true);
    right_turn_motor.init(RIGHT_TURN_PWM, true);
    left_wheelpod_encoder.init(LEFT_WHEELPOD_ENCODER_ID, MULTIPLEXER_0_ID, LEFT_WHEELPOD_ENCODER_START_ANGLE);
    right_wheelpod_encoder.init(RIGHT_WHEELPOD_ENCODER_ID, MULTIPLEXER_0_ID, RIGHT_WHEELPOD_ENCODER_START_ANGLE);
    can_controller.init();
    leftWheelpodAngleSetpoint = 0.0;
    rightWheelpodAngleSetpoint = 0.0;
    leftWheelpodAngle = 0;
    rightWheelpodAngle = 0;
    turnMotorEffort = 0;
    driveSpeed = 0.0;
    poseStepX = 0.0;
    poseStepY = 0.0;
    poseStepTheta = 0.0;
    state = 0;

    for (int i = 0; i < 2; i++) {
        wheelDisplacement[i] = 0.0;
    } 

    cosThetaL = 0.0;
    sinThetaL = 0.0;
    cosThetaR = 0.0;
    sinThetaR = 0.0;

    for (int i = 0; i < 2; i++) {
        newPostion0[i] = 0;
        newPostion1[i] = 0;
        newPostion2[i] = 0;
        newPostion3[i] = 0;
    }

    angleFromWheel0 = 0.0;
    angleFromWheel1 = 0.0;
    angleFromWheel2 = 0.0;
    angleFromWheel3 = 0.0;
}

void Drivetrain::enable() {
    state = DRIVE_STRAIGHT;
    driveSpeed = 0;
}

void Drivetrain::disable() {
    // Set wheel speeds to 0 before disabling
    can_controller.cutCurrent();
    state = 0;
}

void Drivetrain::setState(int newState){
    state = newState;
}

int Drivetrain::getState() {
    return state;
}

void Drivetrain::loop() {

    // Always Get Data
    can_controller.updateMotorSpeeds();
    leftWheelpodAngle = left_wheelpod_encoder.getAngle();
    rightWheelpodAngle = right_wheelpod_encoder.getAngle();

    switch(state){
        case DISABLED:
            setLeftWheelpodAngleSetpoint(0.0);
            setRightWheelpodAngleSetpoint(0.0);
            can_controller.cutCurrent();
            leftTurnI = 0;
            rightTurnI = 0;
            break;
        case DRIVE_STRAIGHT:
            setLeftWheelpodAngleSetpoint(0.0);
            setRightWheelpodAngleSetpoint(0.0);
            setWheelSpeeds(driveSpeed, driveSpeed, driveSpeed, driveSpeed);
            break;
        case POINT_TURN:
            setLeftWheelpodAngleSetpoint(0.7853);
            setRightWheelpodAngleSetpoint(-0.7853);
            setWheelSpeeds(-driveSpeed, -driveSpeed, driveSpeed, driveSpeed);
            break;
        case ICC_TURN:
            // turnICC(yICC, driveSpeed);
            setLeftWheelpodAngleSetpoint(1.5708);
            setRightWheelpodAngleSetpoint(-1.5708);
            setWheelSpeeds(-driveSpeed, driveSpeed, -driveSpeed, driveSpeed);
            break;
        case LEFT_WHEELPOD_RECOVERY:
            can_controller.cutCurrent();
            leftTurnI = 0;(current control)
            rightTurnI = 0;
            break;
        case RIGHT_WHEELPOD_RECOVERY:
            can_controller.cutCurrent();
            leftTurnI = 0;
            rightTurnI = 0;
            break;
    }
    
    // If enabled, control the steering motors
    if (state != DISABLED && state != LEFT_WHEELPOD_RECOVERY && state != RIGHT_WHEELPOD_RECOVERY) {

        // Calculate Errors
        left_turn_motor_error = leftWheelpodAngle - leftWheelpodAngleSetpoint;
        right_turn_motor_error = rightWheelpodAngle - rightWheelpodAngleSetpoint;

        // Integrate Errors
        leftTurnI += left_turn_motor_error;
        rightTurnI += right_turn_motor_error;

        // Contrain Errors with Sumcap
        leftTurnI = constrain(leftTurnI, -TURN_I_SUMCAP, TURN_I_SUMCAP);
        rightTurnI = constrain(rightTurnI, -TURN_I_SUMCAP, TURN_I_SUMCAP);

        // Turn Motors PI Control
        left_turn_motor.setEffort24(int((left_turn_motor_error * TURN_MOTOR_KP) + (TURN_MOTOR_KI * leftTurnI)));
        right_turn_motor.setEffort24(int((right_turn_motor_error * TURN_MOTOR_KP) + (TURN_MOTOR_KI * rightTurnI)));

    } else if (state == LEFT_WHEELPOD_RECOVERY) {
        left_turn_motor.setEffort24(driveSpeed * RECOVERY_MULTIPLIER);
        right_turn_motor.setEffort24(0);

    } else if (state == RIGHT_WHEELPOD_RECOVERY) {
        left_turn_motor.setEffort24(0);
        right_turn_motor.setEffort24(driveSpeed * RECOVERY_MULTIPLIER);
    }
}

void Drivetrain::stepOdom(){

    // Calculate distances travelled by each wheel in the previous timestep
    wheelDisplacement[0] = can_controller.getDisplacement(0) * M_PER_TICK * ODOM_CORRECTION_FACTOR;
    wheelDisplacement[1] = can_controller.getDisplacement(1) * M_PER_TICK * ODOM_CORRECTION_FACTOR;
    wheelDisplacement[2] = can_controller.getDisplacement(2) * M_PER_TICK * ODOM_CORRECTION_FACTOR;
    wheelDisplacement[3] = can_controller.getDisplacement(3) * M_PER_TICK * ODOM_CORRECTION_FACTOR;

    // Pre calculate trig of wheel angles
    cosThetaL = cos(getLeftWheelpodAngle());
    sinThetaL = sin(getLeftWheelpodAngle());
    cosThetaR = cos(getRightWheelpodAngle());
    sinThetaR = sin(getRightWheelpodAngle());

    // Calculate estimated new wheel positions using the wheel angles and the displacements
    newPostion0[0] =  ROBOT_LENGTH_M / 2 + cosThetaL * wheelDisplacement[0];
    newPostion0[1] =  ROBOT_WIDTH_M / 2 - sinThetaL * wheelDisplacement[0];
    newPostion1[0] = -ROBOT_LENGTH_M / 2 + cosThetaL * wheelDisplacement[1];
    newPostion1[1] =  ROBOT_WIDTH_M / 2 + sinThetaL * wheelDisplacement[1];
    newPostion2[0] = -ROBOT_LENGTH_M / 2 + cosThetaR * wheelDisplacement[2];
    newPostion2[1] = -ROBOT_WIDTH_M / 2 + sinThetaR * wheelDisplacement[2];
    newPostion3[0] =  ROBOT_LENGTH_M / 2 + cosThetaR * wheelDisplacement[3];
    newPostion3[1] = -ROBOT_WIDTH_M / 2 - sinThetaR * wheelDisplacement[3];

    // Calculate the average position of the new wheel positions (rounded to 4 places)
    poseStepX = float((newPostion0[0] + newPostion1[0] + newPostion2[0] + newPostion3[0]) / 4); 
    poseStepY = float((newPostion0[1] + newPostion1[1] + newPostion2[1] + newPostion3[1]) / 4);

    // Calculate the new angle using the new wheel positions
    // Left side
    // Calculate the new angle using the new wheel positions
    //Wheel 0
    angleFromWheel0 = atan2(newPostion0[1] - poseStepY, newPostion0[0] - poseStepX) - ANGLE_TO_WHEEL_0;
    angleFromWheel1 = atan2(newPostion1[1] - poseStepY, newPostion1[0] - poseStepX) + ANGLE_TO_WHEEL_0 - PI;
    angleFromWheel2 = atan2(newPostion2[1] - poseStepY, newPostion2[0] - poseStepX) - ANGLE_TO_WHEEL_0 + PI;
    angleFromWheel3 = atan2(newPostion3[1] - poseStepY, newPostion3[0] - poseStepX) + ANGLE_TO_WHEEL_0;

    // Add average angle to output (rounded to 4 places)
    poseStepTheta = float((angleFromWheel0 + angleFromWheel1 + angleFromWheel2 + angleFromWheel3) / 4);
}

void Drivetrain::setWheelSpeeds(float sp0, float sp1, float sp2, float sp3) {
    can_controller.setSpeed(-sp0, -sp1, sp2, sp3);
}

void Drivetrain::setWheelSpeeds(float speedL, float speedR){
    //allows for setting left and right wheel speeds
    setWheelSpeeds(speedL, speedL, speedR, speedR);
}

void Drivetrain::turnICC(float yICC, float topSpeed) {
    // Calculate angles
    
    // float thetaR = atan2((ROBOT_LENGTH_CM/2), (- yICC - (ROBOT_WIDTH_CM/2)));
    // float thetaL = atan2((ROBOT_LENGTH_CM/2), (- yICC + (ROBOT_WIDTH_CM/2)));
    float thetaR = atan2((ROBOT_WIDTH_M / 2) + yICC,   ROBOT_LENGTH_M / 2) - HALF_PI;
    //note: right angle is inverted from our calculations, this math assumes turning the front wheels inwards is positive theta for L and R. If confused, ask Ian
    float thetaL = HALF_PI - atan2((ROBOT_WIDTH_M / 2) - yICC,   ROBOT_LENGTH_M / 2);

    //limit angles to be from -pi/2 to pi/2
    if (thetaR > HALF_PI){
        thetaR -= PI;
    }

    else if(thetaR < -HALF_PI){
        thetaR += PI;
    }

    if (thetaL > HALF_PI){
        thetaL -= PI;
    }

    else if(thetaL < -HALF_PI){
        thetaL += PI;
    }

    float radiusR = sqrt(pow((double)ROBOT_LENGTH_M / 2, 2) + pow(((double)ROBOT_WIDTH_M/2) + yICC, 2));
    
    float radiusL = sqrt(pow((double)ROBOT_LENGTH_M / 2, 2) + pow(((double)ROBOT_WIDTH_M/2) - yICC, 2));

    float speedL = topSpeed;
    float speedR = topSpeed;

    //Adjust speeds to match differential wheel speeds
    if(radiusL > radiusR){
        speedR *= radiusR / radiusL;
    }
    if(radiusR > radiusL){
        speedL *= radiusL / radiusR;
    }

    //correct speeds in case icc is between wheels
    if(0 <= yICC && yICC < ROBOT_WIDTH_M / 2){
        speedL *= -1;
    }
    if(0 > yICC && yICC > ROBOT_WIDTH_M / -2){
        speedR *= -1;
    }

    setLeftWheelpodAngleSetpoint(thetaL);
    setRightWheelpodAngleSetpoint(thetaR);
    setWheelSpeeds(speedL, speedR);
    
}

float Drivetrain::getSpeed(int motorId) {
    return can_controller.getSpeed(motorId);
}

float Drivetrain::getRealSpeed(int motorId) {
    return can_controller.getRealSpeed(motorId);
}

float Drivetrain::getLeftWheelpodAngle() {
    return left_wheelpod_encoder.getAngle();
}

float Drivetrain::getRightWheelpodAngle() {
    return right_wheelpod_encoder.getAngle();
}

void Drivetrain::setLeftWheelpodAngleSetpoint(float newSetPoint) {
    leftWheelpodAngleSetpoint = newSetPoint;
}

void Drivetrain::setRightWheelpodAngleSetpoint(float newSetPoint) {
    rightWheelpodAngleSetpoint = newSetPoint;
}

float Drivetrain::getLeftWheelpodAngleSetpoint() {
    return leftWheelpodAngleSetpoint;
}

float Drivetrain::getRightWheelpodAngleSetpoint() {
    return rightWheelpodAngleSetpoint;
}

void Drivetrain::setAngle(bool angle) {
    isAngled = angle;
}

bool Drivetrain::isEnabled() {
    return (state != DISABLED);
}

void Drivetrain::setDriveSpeed(float speed) {
    driveSpeed = speed;
}

void Drivetrain::setYICC(float y){
    yICC = y;
}

float Drivetrain::getDriveSpeed() {
    return driveSpeed;
}

String Drivetrain::getSums() {
    return can_controller.getSums();
}

float Drivetrain::getPoseStepX() {
    return poseStepX;
}

float Drivetrain::getPoseStepY() {
    return poseStepY;
}

float Drivetrain::getPoseStepTheta() {
    return poseStepTheta;
}
