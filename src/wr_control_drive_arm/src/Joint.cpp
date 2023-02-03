#include "Joint.hpp"
#include "ros/node_handle.h"
#include "std_msgs/Float64.h"

using std::literals::string_literals::operator""s;

Joint::Joint(std::string name,
             std::function<double()> positionMonitor,
             std::function<void(double)> motorSpeedDispatcher,
             ros::NodeHandle node)
    : name{std::move(name)},
      positionMonitor{std::move(positionMonitor)},
      motorSpeedDispatcher{std::move(motorSpeedDispatcher)},
      executeMotion{false},
      controlLoopUpdateTimer{node.createTimer(ros::Rate{FEEDBACK_UPDATE_FREQUENCY_HZ}, &Joint::onFeedbackUpdateEvent, this, false, false)},
      controlLoopOutputSubscriber{node.subscribe("/control/pid/"s + name + "/output", 1, &Joint::onControlLoopOutput, this)},
      controlLoopSetpointPublisher{node.advertise<std_msgs::Float64>("/control/pid"s + name + "/setpoint", 1)},
      controlLoopFeedbackPublisher{node.advertise<std_msgs::Float64>("/control/pid"s + name + "/feedback", 1)} {}

void Joint::setTarget(double target) {
    this->target = target;
    executeMotion = true;
    controlLoopUpdateTimer.start();
}

auto Joint::hasReachedTarget() const -> bool {} // TODO: IMPLEMENT ME

auto Joint::getName() const -> std::string {
    return name;
}

void Joint::stop() {
    executeMotion = false;
    controlLoopUpdateTimer.stop();
    motorSpeedDispatcher(0);
}

void Joint::onControlLoopOutput(const std_msgs::Float64::ConstPtr &msg) {
    motorSpeedDispatcher(executeMotion ? msg->data : 0);
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