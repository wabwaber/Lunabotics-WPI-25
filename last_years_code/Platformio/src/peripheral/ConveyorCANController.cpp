#include "ConveyorCANController.h"
#include "robotMap.h"

ConveyorCANController::ConveyorCANController() : mcp2515(MCP_CS) {}

void ConveyorCANController::init() {
    mcp2515.reset();
    mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
    mcp2515.setNormalMode();

    lastTime = 0;
    prevAngle = 0;
    error = 0.0;
    prevError = 0.0;
    speedSet = 0.0;
    speed = 0.0;
    realSpeed = 0.0;
    sum = 0.0;
    setCurrent = 0;
    speedSetpoint = 0.0;
    displacement = 0;
    motorCurrent0 = 0;
    motor0Angle = 0;
    motor0deltaAngle = 0;
    time = 0;
    motor0Speed = 0.0;

    motor0Encoder.init(CONVEYOR_ENCODER_ID, CONVEYOR_MULTIPLEXER_ID);
}

void ConveyorCANController::setMotorCurrent(int current){
    canMsgOut.can_id = 0x1FF;
    canMsgOut.can_dlc = 2;

    if(abs(current) > 100) {
        motorCurrent0 = current;
    }
    else {
        motorCurrent0 = 0;
    }

    // int motorCurrent0 = constrain(setCurrent, -MAX_MOTOR_CURRENT, MAX_MOTOR_CURRENT);

    canMsgOut.data[0] = (char)(motorCurrent0 / 256);
    canMsgOut.data[1] = (char)(motorCurrent0 % 256);

    mcp2515.sendMessage(&canMsgOut);
}

void ConveyorCANController::updateMotorSpeeds() {

    motor0Angle = motor0Encoder.getRawAngle();

    time = millis();

    deltaTime = time - lastTime;

    motor0deltaAngle = prevAngle - motor0Angle;

    if (motor0deltaAngle > 2048) {
        motor0deltaAngle -= 4096;
    } else if (motor0deltaAngle < -2048) {
        motor0deltaAngle += 4096;
    }

    displacement += motor0deltaAngle;

    motor0Speed = motor0deltaAngle / deltaTime;

    realSpeed = motor0Speed;

    speed = mf0.filter(motor0Speed);

    // Update all last timestep values
    lastTime = time;
    prevAngle = motor0Angle;
    
}

float ConveyorCANController::getSpeed() {
    return speed;
}

float ConveyorCANController::getRealSpeed() {
    return realSpeed;
}

void ConveyorCANController::cutCurrent() {
    setCurrent = 0;
    setMotorCurrent(setCurrent);
}

int ConveyorCANController::getCurrent() {
    return motorCurrent0;
}

int ConveyorCANController::getRawAngle() {
    return motor0Angle;
}