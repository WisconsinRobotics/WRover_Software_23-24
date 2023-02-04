#include "SingleEncoderJointPositionMonitor.hpp"
#include "MathUtil.hpp"
#include "RoboclawChannel.hpp"
#include "ros/node_handle.h"
#include "ros/this_node.h"

using std::literals::string_literals::operator""s;
using MathUtil::RADIANS_PER_ROTATION;

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const std::string &controllerName,
    RoboclawChannel channel,
    EncoderConfiguration config,
    ros::NodeHandle node)
    : my_counter{SingleEncoderJointPositionMonitor::counter},
      countsPerRotation{config.countsPerRotation},
      offset{config.offset},
      encoderSubscriber{
          node.subscribe(
              "/hsi/roboclaw/"s + controllerName + "/enc/" + (channel == RoboclawChannel::A ? "left" : "right"),
              1,
              &SingleEncoderJointPositionMonitor::onEncoderReceived,
              this)} {
    ++SingleEncoderJointPositionMonitor::counter;
}

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    const SingleEncoderJointPositionMonitor &other)
    : my_counter{SingleEncoderJointPositionMonitor::counter},
      countsPerRotation(other.countsPerRotation),
      offset(other.offset),
      encoderSubscriber{other.encoderSubscriber},
      position{other.position.load()} { ++SingleEncoderJointPositionMonitor::counter; }

SingleEncoderJointPositionMonitor::SingleEncoderJointPositionMonitor(
    SingleEncoderJointPositionMonitor &&other) noexcept
    : my_counter{SingleEncoderJointPositionMonitor::counter},
      countsPerRotation(other.countsPerRotation),
      offset(other.offset),
      encoderSubscriber{other.encoderSubscriber},
      position{other.position.load()} { ++SingleEncoderJointPositionMonitor::counter; }

auto SingleEncoderJointPositionMonitor::operator()() -> double {
    std::cout << ros::this_node::getName() << " (" << my_counter << ") READ POS : " << position << std::endl;
    return position;
}

void SingleEncoderJointPositionMonitor::onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg) {
    auto enc = msg->data;
    auto rotations = MathUtil::corrMod(static_cast<double>(enc - offset), countsPerRotation) / countsPerRotation;
    position = MathUtil::corrMod(rotations * RADIANS_PER_ROTATION + M_PI, RADIANS_PER_ROTATION) - M_PI;
    std::cout << ros::this_node::getName() << " (" << my_counter << ") READ POS : " << position << std::endl;
}
