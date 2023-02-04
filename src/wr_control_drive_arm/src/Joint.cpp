#include "Joint.hpp"
#include "MathUtil.hpp"
#include <numbers>
#include <ros/node_handle.h>
#include <std_msgs/Float64.h>

using std::literals::string_literals::operator""s;
using MathUtil::RADIANS_PER_ROTATION;

Joint::Joint(std::string name,
             std::function<double()> positionMonitor,
             std::function<void(double)> motorSpeedDispatcher,
             ros::NodeHandle node)
    : name{std::move(name)},
      positionMonitor{std::move(positionMonitor)},
      motorSpeedDispatcher{std::move(motorSpeedDispatcher)},
      executeMotion{false},
      controlLoopUpdateTimer{node.createTimer(ros::Rate{FEEDBACK_UPDATE_FREQUENCY_HZ}, &Joint::onFeedbackUpdateEvent, this, false, false)},
      controlLoopOutputSubscriber{node.subscribe("/control/arm/pid/"s + this->name + "/output", 1, &Joint::onControlLoopOutput, this)},
      controlLoopSetpointPublisher{node.advertise<std_msgs::Float64>("/control/arm/pid/"s + this->name + "/setpoint", 1)},
      controlLoopFeedbackPublisher{node.advertise<std_msgs::Float64>("/control/arm/pid/"s + this->name + "/feedback", 1)} {}

void Joint::setTarget(double target, double maxSpeed) {
    this->target = target;
    this->maxSpeed = maxSpeed;
    executeMotion = true;
    controlLoopUpdateTimer.start();
}

auto Joint::hasReachedTarget() const -> bool {
    // Copy-on-read to avoid inconsistencies on asynchronous change
    auto currentTarget = target.load();

    // Compute the upper and lower bounds in the finite encoder space
    auto lBound = MathUtil::corrMod(currentTarget - JOINT_TOLERANCE_RADIANS, RADIANS_PER_ROTATION);
    auto uBound = MathUtil::corrMod(currentTarget + JOINT_TOLERANCE_RADIANS, RADIANS_PER_ROTATION);
    auto position = MathUtil::corrMod(positionMonitor(), RADIANS_PER_ROTATION);
    // If the computed lower bound is lower than the upper bound, perform the computation normally
    if (lBound < uBound)
        return position <= uBound && position >= lBound;
    // Otherwise, check if the value is outside either bound and negate the response
    return position <= uBound || position >= lBound;
}

auto Joint::getName() const -> std::string {
    return name;
}

void Joint::stop() {
    executeMotion = false;
    controlLoopUpdateTimer.stop();
    motorSpeedDispatcher(0);
}

void Joint::onControlLoopOutput(const std_msgs::Float64::ConstPtr &msg) {
    auto cappedPowerUnsigned{std::min(std::abs(msg->data), std::abs(maxSpeed))};
    double cappedPower{0};
    if (msg->data != 0)
        cappedPower = (std::abs(msg->data) / msg->data) * cappedPowerUnsigned;
    motorSpeedDispatcher(executeMotion ? cappedPower : 0);
}

void Joint::onFeedbackUpdateEvent(const ros::TimerEvent &event) {
    // Setpoint
    std_msgs::Float64 setpointMsg;
    setpointMsg.data = target;
    controlLoopSetpointPublisher.publish(setpointMsg);

    // Feedback
    std_msgs::Float64 feedbackMsg;
    feedbackMsg.data = positionMonitor();
    controlLoopFeedbackPublisher.publish(feedbackMsg);
}