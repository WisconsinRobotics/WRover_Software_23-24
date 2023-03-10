#ifndef MOTOR_H
#define MOTOR_H

#include "RoboclawChannel.hpp"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include <string>
#include <optional>

class Motor {
public:
    Motor(const std::string &controllerName, RoboclawChannel channel, ros::NodeHandle node);
    void setSpeed(double speed);
    auto isOverCurrent() const -> bool;

private:

    static constexpr float STALL_THRESHOLD_TIME{0.5F};
    static constexpr int OVER_CURRENT_QUEUE_SIZE{25};


    void setCurrentStatus(const std_msgs::Bool::ConstPtr &msg); 

    ros::Publisher motorSpeedPublisher;
    ros::Subscriber currentOverLimitSubscriber;
    std::optional<ros::Time> beginStallTime;
};

<<<<<<< HEAD
#endif
=======
#endif
>>>>>>> b5ba58849ac6bdf12ff8ef8a27746c2fa6813dbd
