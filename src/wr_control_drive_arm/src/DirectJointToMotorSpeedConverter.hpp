#ifndef DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"
#include <memory>

class DirectJointToMotorSpeedConverter {
public:
    explicit DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor);
    void operator()(double speed);

    DirectJointToMotorSpeedConverter(const DirectJointToMotorSpeedConverter &) = default;
    auto operator=(const DirectJointToMotorSpeedConverter &) -> DirectJointToMotorSpeedConverter & = delete;
    DirectJointToMotorSpeedConverter(DirectJointToMotorSpeedConverter &&) = default;
    auto operator=(DirectJointToMotorSpeedConverter &&) -> DirectJointToMotorSpeedConverter & = delete;
    ~DirectJointToMotorSpeedConverter() = default;

private:
    const std::shared_ptr<Motor> outputMotor;
};

#endif