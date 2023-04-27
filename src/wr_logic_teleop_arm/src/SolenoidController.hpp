// Header guard
#ifndef SOLENOID_CONTROLLER_HPP
#define SOLENOID_CONTROLLER_HPP

#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include <fstream>

class SolenoidController {
public:
    explicit SolenoidController(ros::NodeHandle&);

    void extendSolenoid(const std_msgs::Bool::ConstPtr& msg);
    
    ~SolenoidController();
    SolenoidController(const SolenoidController&) = delete;
    auto operator=(const SolenoidController&) -> SolenoidController& = delete;
    SolenoidController(SolenoidController&&) = delete;
    auto operator=(SolenoidController&&) -> SolenoidController& = delete;

private:

    ros::Subscriber extendYSub;
    bool yPressed;

    std::ofstream file;
    std::ofstream fileUnexport;
};

#endif