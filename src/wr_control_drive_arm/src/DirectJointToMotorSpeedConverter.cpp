#include "DirectJointToMotorSpeedConverter.hpp"

DirectJointToMotorSpeedConverter::DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor, MotorSpeedDirection direction)
    : outputMotor{std::move(outputMotor)},
      direction{direction} {}

void DirectJointToMotorSpeedConverter::operator()(double speed) {
    double actualSpeed{0}; // In event of enum error, stop the motor
    switch (direction) {
    case MotorSpeedDirection::FORWARD:
        actualSpeed = speed;
        break;
    case MotorSpeedDirection::REVERSE:
        actualSpeed = -speed;
        break;
    }
    outputMotor->setSpeed(actualSpeed);
}
