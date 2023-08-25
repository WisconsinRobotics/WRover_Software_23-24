#include "SingleEncoderJointPositionMonitor.hpp"
#include "MathUtil.hpp"
#include "RoboclawChannel.hpp"
#include "ros/node_handle.h"

using std::literals::string_literals::operator""s;
using MathUtil::RADIANS_PER_ROTATION;

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const std::string &controllerName,
    RoboclawChannel channel,
    EncoderConfiguration eConfig,
    MotorConfiguration mConfig,
    ros::NodeHandle node)
    : countsPerRotation{eConfig.countsPerRotation},
      offset{eConfig.offset},
      gearRatio{mConfig.gearRatio},
      position{0},
      encoderSubscriber{
          std::make_shared<ros::Subscriber>(node.subscribe(
              "/hsi/roboclaw/"s + controllerName + "/enc/" + (channel == RoboclawChannel::A ? "left" : "right"),
              1,
              &SingleEncoderJointPositionMonitor::onEncoderReceived,
              this))} {}

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const SingleEncoderJointPositionMonitor &other)
    : countsPerRotation(other.countsPerRotation),
      offset(other.offset),
      gearRatio{other.getGearRatio()},
      encoderSubscriber{other.encoderSubscriber},
      position{other.position.load()} {}

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    SingleEncoderJointPositionMonitor &&other) noexcept
    : countsPerRotation(other.countsPerRotation),
      offset(other.offset),
      gearRatio(other.getGearRatio()),
      encoderSubscriber{std::move(other.encoderSubscriber)},
      position{other.position.load()} {}

auto SingleEncoderJointPositionMonitor::operator()() const -> double {
    return position;
}

void SingleEncoderJointPositionMonitor::onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg) {
    auto enc = static_cast<double>(msg->data);
    auto rotations = MathUtil::corrMod(enc - offset, countsPerRotation) / countsPerRotation;
    position = MathUtil::corrMod(rotations * RADIANS_PER_ROTATION + M_PI, RADIANS_PER_ROTATION) - M_PI;
}

auto SingleEncoderJointPositionMonitor::getCountsPerRotation() const -> int32_t {
    return this->countsPerRotation;
}

auto SingleEncoderJointPositionMonitor::getGearRatio() const -> double {
    return this->gearRatio;
}
