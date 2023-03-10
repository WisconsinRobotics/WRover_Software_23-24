#ifndef SINGLE_ENCODER_JOINT_POSITION_MONITOR_H
#define SINGLE_ENCODER_JOINT_POSITION_MONITOR_H

#include "RoboclawChannel.hpp"
#include <cstdint>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt32.h>
#include <string>

struct EncoderConfiguration {
    int32_t countsPerRotation;
    int32_t offset;
};

class SingleEncoderJointPositionMonitor {
public:
    SingleEncoderJointPositionMonitor(const std::string &controllerName, RoboclawChannel channel, EncoderConfiguration config, ros::NodeHandle node);
    auto operator()() -> double;

    SingleEncoderJointPositionMonitor(const SingleEncoderJointPositionMonitor &);
    auto operator=(const SingleEncoderJointPositionMonitor &) -> SingleEncoderJointPositionMonitor & = delete;
    SingleEncoderJointPositionMonitor(SingleEncoderJointPositionMonitor &&) noexcept;
    auto operator=(SingleEncoderJointPositionMonitor &&) -> SingleEncoderJointPositionMonitor & = delete;
    ~SingleEncoderJointPositionMonitor() = default;

private:
    void onEncoderReceived(const std_msgs::UInt32::ConstPtr &msg);

    std::atomic<double> position;
    std::shared_ptr<ros::Subscriber> encoderSubscriber;
    const int32_t countsPerRotation;
    const int32_t offset;
};

#endif