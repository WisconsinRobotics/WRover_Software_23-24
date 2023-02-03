#include "DirectJointToMotorSpeedConverter.hpp"

DirectJointToMotorSpeedConverter::DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor)
    : outputMotor{std::move(outputMotor)} {}

void DirectJointToMotorSpeedConverter::operator()(double speed) {
    outputMotor->setSpeed(speed);
}
