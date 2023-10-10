/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Declarations for the model of a @ref Motor
 *
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "RoboclawChannel.hpp"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include <optional>
#include <string>

/**
 * @brief An abstraction for a motor
 * @details This might be more useful if it was defined/exported by wroboclaw
 *
 */
class Motor {
public:
    /**
     * @brief Construct a new Motor object
     *
     * @param controllerName The name of the wroboclaw controller ('aux0', 'aux1', ...)
     * @param channel The channel of the motor on the Roboclaw controller
     * @param node The node definition, used to make ROS objects
     */
    Motor(const std::string &controllerName, RoboclawChannel channel, ros::NodeHandle node);
    /**
     * @brief Set the speed of the motor
     *
     * @param speed The new speed of the motor
     */
    void setSpeed(double speed);
    /**
     * @brief Returns whether or not a motor is drawing more current than its predefined limit.
     * This includes a hysteresis timeout to prevent inrush current false positives.
     *
     * @return true The motor is currently overcurrent.  Action should be taken to decrease the current draw to avoid hardware damage.
     * @return false The motor is not overcurrent.
     */
    [[nodiscard]] auto isOverCurrent() const -> bool;

private:
    /// How long the motor must be over the current limit before we report that it's overcurrent
    static constexpr float STALL_THRESHOLD_TIME{2.0F};
    /// How many messages to remember for the overcurrent topic
    static constexpr int OVER_CURRENT_QUEUE_SIZE{25};

    /**
     * @brief Listens to the overcurrent status of the motor
     *
     * @param msg The ROS message for whether or not the motor is over its current limit (no hysteresis timeout)
     */
    void setCurrentStatus(const std_msgs::Bool::ConstPtr &msg);

    /// Publishes motor speed (int16_t)
    ros::Publisher motorSpeedPublisher;
    /// Listens for overcurrent status
    ros::Subscriber currentOverLimitSubscriber;
    /// When the current overcurrent timer started
    std::optional<ros::Time> beginStallTime;
};

#endif
/// @}
