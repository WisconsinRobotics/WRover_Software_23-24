#ifndef SINGLE_ENCODER_JOINT_POSITION_MONITOR_H
#define SINGLE_ENCODER_JOINT_POSITION_MONITOR_H

#include "RoboclawChannel.hpp"
#include <cstdint>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt32.h>
#include <string>

struct EncoderConfiguration {
    uint32_t countsPerRotation;
    uint32_t offset;
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

    inline static std::size_t counter{0};

    const std::size_t my_counter;

    std::atomic<double> position;
    ros::Subscriber encoderSubscriber;
    const uint32_t countsPerRotation;
    const uint32_t offset;
};

#endif