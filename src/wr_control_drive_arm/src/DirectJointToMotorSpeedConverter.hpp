/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Header to define @ref DirectJointToMotorSpeedConverter and @ref MotorSpeedDirection  objects
 */

#ifndef DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIRECT_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"
#include <memory>

/**
 * @brief Enumerates orientation of positive power with respect to positive encoder counting
 *
 */
enum class MotorSpeedDirection {
    FORWARD,
    REVERSE
};

/**
 * @brief Converts @ref Joint speed to @ref Motor speed
 *
 */
class DirectJointToMotorSpeedConverter {
public:
    /**
     * @brief Construct a new Direct Joint To Motor Speed Converter object
     *
     * @param outputMotor The motor driving the @ref Joint
     * @param direction Which way the motor has to spin to get positive encoder counting
     */
    DirectJointToMotorSpeedConverter(std::shared_ptr<Motor> outputMotor, MotorSpeedDirection direction);
    /**
     * @brief Set the speed of the motor driving the joint
     *
     * @param speed The speed of the motor
     */
    void operator()(double speed);

    /**
     * @brief Copy constructor
     *
     */
    DirectJointToMotorSpeedConverter(const DirectJointToMotorSpeedConverter &) = default;
    auto operator=(const DirectJointToMotorSpeedConverter &) -> DirectJointToMotorSpeedConverter & = delete;
    /**
     * @brief Move Constructor
     *
     */
    DirectJointToMotorSpeedConverter(DirectJointToMotorSpeedConverter &&) = default;
    auto operator=(DirectJointToMotorSpeedConverter &&) -> DirectJointToMotorSpeedConverter & = delete;
    ~DirectJointToMotorSpeedConverter() = default;

private:
    /// Which way the motor has to spin to get positive encoder counting
    MotorSpeedDirection direction;
    /// The motor driving the @ref Joint
    std::shared_ptr<Motor> outputMotor;
};

#endif
/// @}
