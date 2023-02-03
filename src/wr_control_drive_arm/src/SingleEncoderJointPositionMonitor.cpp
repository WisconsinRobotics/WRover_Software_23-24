#include "SingleEncoderJointPositionMonitor.hpp"
#include "MathUtil.hpp"
#include "RoboclawChannel.hpp"
#include <numbers>

using std::literals::string_literals::operator""s;
using MathUtil::RADIANS_PER_ROTATION;

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const std::string &controllerName,
    RoboclawChannel channel,
    EncoderConfiguration config,
    ros::NodeHandle node)
    : countsPerRotation{config.countsPerRotation},
      offset{config.offset},
      encoderSubscriber{
          node.subscribe(
              "/hsi/roboclaw/"s + controllerName + "/enc/" + (channel == RoboclawChannel::A ? "left" : "right"),
              1,
              &SingleEncoderJointPositionMonitor::onEncoderReceived,
              this)} {}

auto SingleEncoderJointPositionMonitor::operator()() -> double {
    return position;
}

void SingleEncoderJointPositionMonitor::onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg) {
    auto enc = msg->data;
    double rotations = MathUtil::corrMod(static_cast<double>(enc - offset), countsPerRotation) / countsPerRotation;
    position = MathUtil::corrMod(rotations * RADIANS_PER_ROTATION, RADIANS_PER_ROTATION) - std::numbers::pi;
}
