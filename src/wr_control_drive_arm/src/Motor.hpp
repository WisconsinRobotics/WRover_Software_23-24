#ifndef MOTOR_H
#define MOTOR_H

#include "RoboclawChannel.hpp"
#include "ros/publisher.h"
#include <string>

class Motor {
public:
    Motor(const std::string &controllerName, RoboclawChannel channel, ros::NodeHandle node);
    void setSpeed(double speed);
    auto isOverCurrent() -> bool;

private:
    ros::Publisher motorSpeedPublisher;
};

#endif