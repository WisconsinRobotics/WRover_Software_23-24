#include "Motor.hpp"
#include "RoboclawChannel.hpp"
#include "ros/node_handle.h"
#include "std_msgs/Int16.h"
#include <cstdint>
#include <stdexcept>

using std::literals::string_literals::operator""s;

Motor::Motor(const std::string &controllerName, RoboclawChannel channel, ros::NodeHandle node)
    : motorSpeedPublisher{
          node.advertise<std_msgs::Int16>(
              "/hsi/roboclaw/"s + controllerName + "/cmd/"s + (channel == RoboclawChannel::A ? "left" : "right"),
              1)} {}

void Motor::setSpeed(double speed) {
    if (abs(speed) > 1)
        throw std::invalid_argument("speed must be between -1 and 1");

    std_msgs::Int16 powerMsg{};
    powerMsg.data = static_cast<int16_t>(speed * INT16_MAX);
    motorSpeedPublisher.publish(powerMsg);
}

auto Motor::isOverCurrent() -> bool { // NOLINT(readability-convert-member-functions-to-static)
    return false;                     // TODO: IMPLEMENT ME
}