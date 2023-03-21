#ifndef DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"
#include <memory>

enum class MotorSpeedDirection {
    FORWARD,
    REVERSE
};

struct MotorConfiguration {
    double gearRatio;
};

class DirectJointToMotorSpeedConverter {
public:
    DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor, MotorSpeedDirection direction, MotorConfiguration config);
    void operator()(double speed);

    DirectJointToMotorSpeedConverter(const DirectJointToMotorSpeedConverter &) = default;
    auto operator=(const DirectJointToMotorSpeedConverter &) -> DirectJointToMotorSpeedConverter & = delete;
    DirectJointToMotorSpeedConverter(DirectJointToMotorSpeedConverter &&) = default;
    auto operator=(DirectJointToMotorSpeedConverter &&) -> DirectJointToMotorSpeedConverter & = delete;
    ~DirectJointToMotorSpeedConverter() = default;

    [[nodiscard]] auto getGearRatio() const -> double;

private:
    const MotorSpeedDirection direction;
    const std::shared_ptr<Motor> outputMotor;
    const double gearRatio;
};

#endif