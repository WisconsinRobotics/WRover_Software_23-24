#ifndef DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"
#include "SingleEncoderJointPositionMonitor.hpp"
#include <memory>

enum class MotorSpeedDirection {
    FORWARD,
    REVERSE
};

class DirectJointToMotorSpeedConverter {
public:
    DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor, MotorSpeedDirection direction);
    void operator()(double speed);

    DirectJointToMotorSpeedConverter(const DirectJointToMotorSpeedConverter &) = default;
    auto operator=(const DirectJointToMotorSpeedConverter &) -> DirectJointToMotorSpeedConverter & = delete;
    DirectJointToMotorSpeedConverter(DirectJointToMotorSpeedConverter &&) = default;
    auto operator=(DirectJointToMotorSpeedConverter &&) -> DirectJointToMotorSpeedConverter & = delete;
    ~DirectJointToMotorSpeedConverter() = default;

private:
    const MotorSpeedDirection direction;
    const std::shared_ptr<Motor> outputMotor;
};

#endif