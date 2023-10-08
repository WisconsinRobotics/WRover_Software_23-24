/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Header to define @ref DifferentialJointToMotorSpeedConverter objects
 */

#ifndef DIFFERENTIAL_JOINT_TO_MOTOR_SPEED_CONVERTER_H
#define DIFFERENTIAL_JOINT_TO_MOTOR_SPEED_CONVERTER_H

#include "Motor.hpp"

/**
 * @brief Adapts joint speeds to motor speeds for the differential joint
 *
 */
class DifferentialJointToMotorSpeedConverter {
public:
    /**
     * @brief Construct a new Differential Joint To Motor Speed Converter object
     *
     * @param leftMotor The 'left'-side @ref Motor of the differential joint
     * @param rightMotor The 'right'-side @ref Motor of the differential joint
     */
    DifferentialJointToMotorSpeedConverter(std::shared_ptr<Motor> leftMotor, std::shared_ptr<Motor> rightMotor);
    /**
     * @brief Set the speed of the pitch @ref Joint
     *
     * @param speed The new speed of the @ref Joint
     */
    void setPitchSpeed(double speed);
    /**
     * @brief Set the speed of the roll @ref Joint
     *
     * @param speed The new speed of the @ref Joint
     */
    void setRollSpeed(double speed);

    /**
     * @brief Copy constructor
     *
     */
    DifferentialJointToMotorSpeedConverter(const DifferentialJointToMotorSpeedConverter &);
    auto operator=(const DifferentialJointToMotorSpeedConverter &) -> DifferentialJointToMotorSpeedConverter & = delete;
    /**
     * @brief Move constructor
     *
     */
    DifferentialJointToMotorSpeedConverter(DifferentialJointToMotorSpeedConverter &&) noexcept;
    auto operator=(DifferentialJointToMotorSpeedConverter &&) -> DifferentialJointToMotorSpeedConverter & = delete;
    ~DifferentialJointToMotorSpeedConverter() = default;

private:
    /// The commanded pitch speed
    std::atomic<double> cachedPitchSpeed;
    /// The commanded roll speed
    std::atomic<double> cachedRollSpeed;

    /// Reference to the left motor
    std::shared_ptr<Motor> leftMotor;
    /// Reference to the right motor
    std::shared_ptr<Motor> rightMotor;

    /// Balance factor between pitch and roll
    static constexpr double AVERAGE_SCALING_FACTOR{1};

    /**
     * @brief Update both motors with speeds derived by the commanded pitch/roll speeds
     *
     */
    void dispatchDifferentialSpeed();
};

#endif
/// @}
