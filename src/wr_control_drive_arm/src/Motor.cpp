#include "Motor.hpp"
#include "RoboclawChannel.hpp"
#include "ros/node_handle.h"
#include "ros/time.h"
#include "std_msgs/Int16.h"
#include <atomic>
#include <cstdint>
#include <optional>
#include <stdexcept>

using std::literals::string_literals::operator""s;

Motor::Motor(const std::string &controllerName, RoboclawChannel channel, ros::NodeHandle node)
    : motorSpeedPublisher{
          node.advertise<std_msgs::Int16>(
              "/hsi/roboclaw/"s + controllerName + "/cmd/"s + (channel == RoboclawChannel::A ? "left" : "right"),
              1)},
      currentOverLimitSubscriber{
          node.subscribe("/hsi/roboclaw/"s + controllerName + "/curr/over_lim/" + (channel == RoboclawChannel::A ? "left" : "right"),
              OVER_CURRENT_QUEUE_SIZE, &Motor::setCurrentStatus, this)} {}

void Motor::setSpeed(double speed) {
    if (abs(speed) > 1)
        throw std::invalid_argument("speed must be between -1 and 1");

    std_msgs::Int16 powerMsg{};
    powerMsg.data = static_cast<int16_t>(speed * INT16_MAX);
    motorSpeedPublisher.publish(powerMsg);
}

void Motor::setCurrentStatus(const std_msgs::Bool::ConstPtr &msg) {
    bool overCurrent = static_cast<bool>(msg->data);
    if (overCurrent && !this->beginStallTime.has_value()) {
        this->beginStallTime = ros::Time::now();
    } else if (!overCurrent && this->beginStallTime.has_value()) {
        this->beginStallTime = std::nullopt;
    }
}

auto Motor::isOverCurrent() const -> bool {
    if (this->beginStallTime.has_value()) {
        return static_cast<bool>((ros::Time::now() - this->beginStallTime.value()).toSec() >= STALL_THRESHOLD_TIME);
    }
    return false;
}
