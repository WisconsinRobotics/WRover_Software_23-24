#ifndef DIFFERENTIAL_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIFFERENTIAL_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"

class DifferentialJointToMotorSpeedConverter {
public:
    DifferentialJointToMotorSpeedConverter(std::shared_ptr<Motor> leftMotor, std::shared_ptr<Motor> rightMotor);
    void setPitchSpeed(double speed);
    void setRollSpeed(double speed);

    DifferentialJointToMotorSpeedConverter(const DifferentialJointToMotorSpeedConverter &);
    auto operator=(const DifferentialJointToMotorSpeedConverter &) -> DifferentialJointToMotorSpeedConverter & = delete;
    DifferentialJointToMotorSpeedConverter(DifferentialJointToMotorSpeedConverter &&) noexcept;
    auto operator=(DifferentialJointToMotorSpeedConverter &&) -> DifferentialJointToMotorSpeedConverter & = delete;
    ~DifferentialJointToMotorSpeedConverter() = default;

private:
    std::atomic<double> cachedPitchSpeed;
    std::atomic<double> cachedRollSpeed;
    std::mutex mutex;

    const std::shared_ptr<Motor> leftMotor;
    const std::shared_ptr<Motor> rightMotor;

    static constexpr double AVERAGE_SCALING_FACTOR{0.5};

    void dispatchDifferentialSpeed();
};

#endif