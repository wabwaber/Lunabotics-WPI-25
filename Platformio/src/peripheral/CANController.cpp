#include "CANController.h"
#include "robotMap.h"

CANController::CANController() : mcp2515(MCP_CS) {}

void CANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    lastTime = 0;

    for (int i = 0; i < 4; i++) {
        prevAngles[i] = 0.0;
        errors[i] = 0.0;
        prevErrors[i] = 0.0;
        setSpeeds[i] = 0.0;
        speeds[i] = 0.0;
        realSpeeds[i] = 0.0;
        sums[i] = 0.0;
        setCurrents[i] = 0;
        speedSetpoints[i] = 0.0;
        displacements[i] = 0;
    }

    motor0Encoder.init(FRONT_LEFT_DRIVE_ENCODER_ID, MULTIPLEXER_0_ID);
    motor1Encoder.init(BACK_LEFT_DRIVE_ENCODER_ID, MULTIPLEXER_0_ID);
    motor2Encoder.init(BACK_RIGHT_DRIVE_ENCODER_ID, MULTIPLEXER_0_ID);
    motor3Encoder.init(FRONT_RIGHT_DRIVE_ENCODER_ID, MULTIPLEXER_0_ID);
}

void CANController::setMotorCurrent() {
    canMsgOut.can_id = 0x200;
    canMsgOut.can_dlc = 8;

    int motorCurrent0 = constrain(setCurrents[0], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
    int motorCurrent1 = constrain(setCurrents[1], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
    int motorCurrent2 = constrain(setCurrents[2], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);
    int motorCurrent3 = constrain(setCurrents[3], -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);

    canMsgOut.data[0] = (char)(motorCurrent0 / 256);
    canMsgOut.data[1] = (char)(motorCurrent0 % 256);
    canMsgOut.data[2] = (char)(motorCurrent1 / 256);
    canMsgOut.data[3] = (char)(motorCurrent1 % 256);
    canMsgOut.data[4] = (char)(motorCurrent2 / 256);
    canMsgOut.data[5] = (char)(motorCurrent2 % 256);
    canMsgOut.data[6] = (char)(motorCurrent3 / 256);
    canMsgOut.data[7] = (char)(motorCurrent3 % 256);

    mcp2515.sendMessage(&canMsgOut);
}

void CANController::speedHandlerPID() {

    // Define Errors for this loop
    errors[0] = setSpeeds[0] - speeds[0];
    errors[1] = setSpeeds[1] - speeds[1];
    errors[2] = setSpeeds[2] - speeds[2];
    errors[3] = setSpeeds[3] - speeds[3];

    // Integrate the error over time
    sums[0] = constrain(sums[0] + errors[0], -SPEED_SUMCAP, SPEED_SUMCAP);
    sums[1] = constrain(sums[1] + errors[1], -SPEED_SUMCAP, SPEED_SUMCAP);
    sums[2] = constrain(sums[2] + errors[2], -SPEED_SUMCAP, SPEED_SUMCAP);
    sums[3] = constrain(sums[3] + errors[3], -SPEED_SUMCAP, SPEED_SUMCAP);

    // PID Here
    if (setSpeeds[0] == 0) {
        setCurrents[0] = 0;
        sums[0] = 0;
    } else if (setSpeeds[0] > 0) {
        setCurrents[0] = BASE_CURRENT + SPEED_KP * errors[0] + SPEED_KI * sums[0];
    } else {
        setCurrents[0] = -BASE_CURRENT + SPEED_KP * errors[0] + SPEED_KI * sums[0];
    }

    if (setSpeeds[1] == 0) {
        setCurrents[1] = 0;
        sums[1] = 0;
    } else if (setSpeeds[1] > 0) {
        setCurrents[1] = BASE_CURRENT + SPEED_KP * errors[1] + SPEED_KI * sums[1];
    } else {
        setCurrents[1] = -BASE_CURRENT + SPEED_KP * errors[1] + SPEED_KI * sums[1];
    }

    if (setSpeeds[2] == 0) {
        setCurrents[2] = 0;
        sums[2] = 0;
    } else if (setSpeeds[2] > 0) {
        setCurrents[2] = BASE_CURRENT + SPEED_KP * errors[2] + SPEED_KI * sums[2];
    } else {
        setCurrents[2] = -BASE_CURRENT + SPEED_KP * errors[2] + SPEED_KI * sums[2];
    }

    if (setSpeeds[3] == 0) {
        setCurrents[3] = 0;
        sums[3] = 0;
    } else if (setSpeeds[3] > 0) {
        setCurrents[3] = BASE_CURRENT + SPEED_KP * errors[3] + SPEED_KI * sums[3];
    } else {
        setCurrents[3] = -BASE_CURRENT + SPEED_KP * errors[3] + SPEED_KI * sums[3];
    }

    prevErrors[0] = errors[0];
    prevErrors[1] = errors[1];
    prevErrors[2] = errors[2];
    prevErrors[3] = errors[3];

    setMotorCurrent();
}

void CANController::updateMotorSpeeds() {
    float motor0Angle = motor0Encoder.getRawAngle();
    float motor1Angle = motor1Encoder.getRawAngle();
    float motor2Angle = motor2Encoder.getRawAngle();
    float motor3Angle = motor3Encoder.getRawAngle();

    long time = millis();

    long deltaTime = time - lastTime;

    float motor0deltaAngle = motor0Angle - prevAngles[0];
    float motor1deltaAngle = motor1Angle - prevAngles[1];
    float motor2deltaAngle = motor2Angle - prevAngles[2];
    float motor3deltaAngle = motor3Angle - prevAngles[3];

    if (motor0deltaAngle > 2048) {
        motor0deltaAngle -= 4096;
    } else if (motor0deltaAngle < -2048) {
        motor0deltaAngle += 4096;
    }

    if (motor1deltaAngle > 2048) {
        motor1deltaAngle -= 4096;
    } else if (motor1deltaAngle < -2048) {
        motor1deltaAngle += 4096;
    }

    if (motor2deltaAngle > 2048) {
        motor2deltaAngle -= 4096;
    } else if (motor2deltaAngle < -2048) {
        motor2deltaAngle += 4096;
    }

    if (motor3deltaAngle > 2048) {
        motor3deltaAngle -= 4096;
    } else if (motor3deltaAngle < -2048) {
        motor3deltaAngle += 4096;
    }

    displacements[0] += motor0deltaAngle;
    displacements[1] += motor1deltaAngle;
    displacements[2] += motor2deltaAngle;
    displacements[3] += motor3deltaAngle;

    float motor0Speed = motor0deltaAngle / deltaTime;
    float motor1Speed = motor1deltaAngle / deltaTime;
    float motor2Speed = motor2deltaAngle / deltaTime;
    float motor3Speed = motor3deltaAngle / deltaTime;

    realSpeeds[0] = motor0Speed;
    realSpeeds[1] = motor1Speed;
    realSpeeds[2] = motor2Speed;
    realSpeeds[3] = motor3Speed;

    speeds[0] = mf0.filter(motor0Speed);
    speeds[1] = mf1.filter(motor1Speed);
    speeds[2] = mf2.filter(motor2Speed);
    speeds[3] = mf3.filter(motor3Speed);

    // Update all last timestep values
    lastTime = time;
    prevAngles[0] = motor0Angle;
    prevAngles[1] = motor1Angle;
    prevAngles[2] = motor2Angle;
    prevAngles[3] = motor3Angle;
    
}

void CANController::setSpeed(float sp0, float sp1, float sp2, float sp3) {
    setSpeeds[0] = sp0;
    setSpeeds[1] = sp1;
    setSpeeds[2] = sp2;
    setSpeeds[3] = sp3;

    speedHandlerPID();
}

float CANController::getSpeed(int motorId) {
    if (motorId == 0 || motorId == 1) {
        return -speeds[motorId];
    }
    return speeds[motorId];
}

float CANController::getRealSpeed(int motorId) {
    if (motorId == 0 || motorId == 1) {
        return -realSpeeds[motorId];
    }
    return realSpeeds[motorId];
}

int CANController::getDisplacement(int motorId){
    if (motorId == 0 || motorId == 1) {
        int output = -displacements[motorId];
        displacements[motorId] = 0;
        return output;
    }
    
    int output = displacements[motorId];
    displacements[motorId] = 0;
    return output;

}

void CANController::cutCurrent() {
    setCurrents[0] = 0;
    setCurrents[1] = 0;
    setCurrents[2] = 0;
    setCurrents[3] = 0;
    setMotorCurrent();
}

String CANController::getSums() {
    return String(sums[0]);
}