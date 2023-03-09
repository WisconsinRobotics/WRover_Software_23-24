// Header guard
#ifndef SOLENOID_CONTROLLER_HPP
#define SOLENOID_CONTROLLER_HPP

#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

class SolenoidController{
public:
    explicit SolenoidController(ros::NodeHandle&);

    void extendSolenoid(const std_msgs::Bool::ConstPtr& msg);

    void checkMessage();

private:
    // You may want to keep the speed ros::Publisher as a member variable to publish
    // from the member methods
    ros::Subscriber extendYSub;
    bool yPressed;
    // If you store a publisher, you may want to construct that publisher in the
    // constructor of the class
    
};

#endif