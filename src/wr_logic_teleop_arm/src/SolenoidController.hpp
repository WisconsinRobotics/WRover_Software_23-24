// Header guard
#ifndef SOLENOID_CONTROLLER_HPP
#define SOLENOID_CONTROLLER_HPP

#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

class SolenoidController {
public:
    explicit SolenoidController(ros::NodeHandle&);

    void extendSolenoid(const std_msgs::Bool::ConstPtr& msg);
    
    ~SolenoidController();

private:

    ros::Subscriber extendYSub;
    bool yPressed;

    
};

#endif