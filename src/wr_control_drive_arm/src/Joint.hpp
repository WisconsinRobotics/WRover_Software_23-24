#ifndef JOINT_H
#define JOINT_H

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "ros/timer.h"
#include "std_msgs/Float64.h"
#include <functional>
#include <string>

class Joint {
public:
    explicit Joint(std::string name, std::function<double()> positionMonitor, std::function<void(double)> motorSpeedDispatcher, ros::NodeHandle node);
    void setTarget(double target, double maxSpeed);
    [[nodiscard]] auto hasReachedTarget() const -> bool;
    [[nodiscard]] auto getName() const -> std::string;
    void stop();

private:
    static constexpr double FEEDBACK_UPDATE_FREQUENCY_HZ{50};
    static constexpr double JOINT_TOLERANCE_RADIANS{3 * M_PI / 180};

    const std::string name;
    const std::function<double()> positionMonitor;
    const std::function<void(double)> motorSpeedDispatcher;

    void onControlLoopOutput(const std_msgs::Float64::ConstPtr &msg);
    void onFeedbackUpdateEvent(const ros::TimerEvent &event);

    std::atomic<double> target;
    std::atomic<double> maxSpeed;
    std::atomic<bool> executeMotion;
    ros::Timer controlLoopUpdateTimer;
    ros::Subscriber controlLoopOutputSubscriber;
    ros::Publisher controlLoopSetpointPublisher;
    ros::Publisher controlLoopFeedbackPublisher;
};

#endif