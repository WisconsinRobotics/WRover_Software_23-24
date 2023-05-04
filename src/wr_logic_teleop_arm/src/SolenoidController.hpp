// Header guard
#ifndef SOLENOID_CONTROLLER_HPP
#define SOLENOID_CONTROLLER_HPP

#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include <fstream>

class SolenoidController {
public:
    explicit SolenoidController(ros::NodeHandle&);
    
    ~SolenoidController();
    SolenoidController(const SolenoidController&) = delete;
    auto operator=(const SolenoidController&) -> SolenoidController& = delete;
    SolenoidController(SolenoidController&&) = delete;
    auto operator=(SolenoidController&&) -> SolenoidController& = delete;

private:
    ros::Subscriber extendYSub;
    bool yPressed;

    std::ofstream file;
    std::ofstream fileExport;
    std::ofstream fileUnexport;

    static constexpr uint32_t MESSAGE_QUEUE_LENGTH{1};
    static constexpr uint16_t pin{6};

    void extendSolenoid(const std_msgs::Bool::ConstPtr& msg);
};

#endif