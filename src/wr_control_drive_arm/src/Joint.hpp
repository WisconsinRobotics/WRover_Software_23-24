/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Header to define @ref Joint objects
 */

#ifndef JOINT_H
#define JOINT_H

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "std_msgs/Float64.h"
#include <functional>
#include <string>

/**
 * @brief Represents a joint that MoveIt can control
 *
 */
class Joint {
public:
    /**
     * @brief Construct a new Joint object
     *
     * @param name The name of the Joint
     * @param positionMonitor A way to get a position of the Joint
     * @param motorSpeedDispatcher A way to set the speed of the Joint
     * @param node The Node Handle required to interface with ROS
     */
    explicit Joint(std::string name, std::function<double()> positionMonitor, std::function<void(double)> motorSpeedDispatcher, ros::NodeHandle node);
    /**
     * @brief Set the target position of this Joint
     *
     * @param target The new target of the Joint
     * @param maxSpeed The maximum speed the Joint to reach its target
     */
    void setTarget(double target, double maxSpeed);
    /**
     * @brief Whether or not a joint has reached its target
     *
     * @return true The joint is at its target
     * @return false The joint is not at its target
     */
    [[nodiscard]] auto hasReachedTarget() const -> bool;
    /**
     * @brief Get the name of the Joint
     *
     * @return std::string The name of the Joint
     */
    [[nodiscard]] auto getName() const -> std::string;
    /**
     * @brief Command the Joint to stop
     *
     */
    void stop();

private:
    /// How quickly to provide feedback on the joint position
    static constexpr double FEEDBACK_UPDATE_FREQUENCY_HZ{50};
    /// How close does the joint have to be before it's 'at' its target
    static constexpr double JOINT_TOLERANCE_RADIANS{3 * M_PI / 180};

    /// The name of the Joint
    std::string name;
    /// How we get the position of the Joint
    std::function<double()> positionMonitor;
    /// How we set the speed of the Joint
    std::function<void(double)> motorSpeedDispatcher;

    /**
     * @brief Update the joint speed based on control loop feedback
     *
     * @param msg The control loop's opinion on how fast the joint should move
     */
    void onControlLoopOutput(const std_msgs::Float64::ConstPtr &msg);
    /**
     * @brief Update the control loop with joint position feedback
     *
     * @param event ROS timer event data, unused
     */
    void onFeedbackUpdateEvent(const ros::TimerEvent &event);

    /// The joint's target position
    std::atomic<double> target{};
    /// The maximum speed the joint can travel at
    std::atomic<double> maxSpeed{};
    /// Whether or not the joint can execute motion
    std::atomic<bool> executeMotion;
    /// Timer to update the control loop
    ros::Timer controlLoopUpdateTimer;
    /// Subscribe to the output of the control loop
    ros::Subscriber controlLoopOutputSubscriber;
    /// Publish the setpoint to the control loop
    ros::Publisher controlLoopSetpointPublisher;
    /// Publish the feedback to the control loop
    ros::Publisher controlLoopFeedbackPublisher;
};

#endif

/// @}
