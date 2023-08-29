/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Declarations for the model of a JointPositionMonitors and required setup objects
 *
 */

#ifndef SINGLE_ENCODER_JOINT_POSITION_MONITOR_H
#define SINGLE_ENCODER_JOINT_POSITION_MONITOR_H

#include "RoboclawChannel.hpp"
#include <cstdint>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt32.h>
#include <string>

/**
 * @brief The configuration for a encoder with relation to a @ref Joint
 *
 */
struct EncoderConfiguration {
    /// How many counts it takes to make a full rotation (accounting for gear ratios between the encoder and @ref Joint output)
    /// @details A 'full rotation' means a full 2*pi of rotation, regardless of what's possible due to hardware stops.
    /// Some extrapolation mustbe necessary to prevent damage to hardware during measurement
    int32_t countsPerRotation;
    /// What's the reading of the encoder when the arm is in the 'calibration' position
    /// @details This should be physically calibrated so the workspace of the @ref Joint does not fall across the rollover point of the encoder.
    int32_t offset;
};

/**
 * @brief The configuration for the @ref Motor of the @ref Joint
 *
 */
struct MotorConfiguration {
    /// The gear ratio between the @ref Joint and its @ref Motor
    double gearRatio;
};

/**
 * @brief Represents a way to use a single encoder to get the position of a single @ref Joint
 *
 */
class SingleEncoderJointPositionMonitor {
public:
    /**
     * @brief Construct a new Single Encoder Joint Position Monitor object
     *
     * @param controllerName The name of the wroboclaw controller ('aux0', 'aux1', ...)
     * @param channel The Roboclaw channel of the encoder.  May or may not correspond to the @ref Motor of the @ref Joint
     * @param eConfig The configuration of the encoder
     * @param mConfig The configuration of the motor.  This should questionably be somewhere else.
     * @param node The ROS node used to create ROS objects
     */
    SingleEncoderJointPositionMonitor(const std::string &controllerName, RoboclawChannel channel, EncoderConfiguration eConfig, MotorConfiguration mConfig, ros::NodeHandle node);
    /**
     * @brief Gets the position of the encoder as a @ref Joint position (i.e. radians)
     *
     * @return double The current position of the encoder, in radians
     */
    auto operator()() const -> double;

    /**
     * @brief Copy constructor
     *
     */
    SingleEncoderJointPositionMonitor(const SingleEncoderJointPositionMonitor &);
    auto operator=(const SingleEncoderJointPositionMonitor &) -> SingleEncoderJointPositionMonitor & = delete;
    /**
     * @brief Move constructor
     *
     */
    SingleEncoderJointPositionMonitor(SingleEncoderJointPositionMonitor &&) noexcept;
    auto operator=(SingleEncoderJointPositionMonitor &&) -> SingleEncoderJointPositionMonitor & = delete;
    ~SingleEncoderJointPositionMonitor() = default;

    /**
     * @brief Get the counts per rotation for the encoder
     *
     * @return int32_t The counts per rotation for the encoder
     */
    [[nodiscard]] auto getCountsPerRotation() const -> int32_t;
    /**
     * @brief Get the gear ratio of the motor
     *
     * @return double The gear ratio of the motor
     */
    [[nodiscard]] auto getGearRatio() const -> double;

private:
    /**
     * @brief ROS callback to update the current encoder count
     *
     * @param msg The ROS message containing the new encoder value
     */
    void onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg);

    /// The current position of the encoder
    std::atomic<double> position;
    /// The subscriber to get encoder data
    /// @details TODO(ben.nowotny): why would this be a std::shared_ptr???
    std::shared_ptr<ros::Subscriber> encoderSubscriber;
    /// The counts per rotation for this encoder
    int32_t countsPerRotation;
    /// The offset for this encoder
    int32_t offset;
    /// The gear ratio for the associated motor
    double gearRatio;
};

#endif

/// @}
