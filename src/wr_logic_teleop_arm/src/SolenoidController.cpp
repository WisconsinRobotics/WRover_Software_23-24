#include "SolenoidController.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

// Again, you may want a constructor here
// For pass-by-value reasons, you may want to take the NodeHandle as an argument
// And construct the Publisher in this constructor, rather that taking the Publisher as 
// an argument.

constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;

SolenoidController::SolenoidController(ros::NodeHandle& n) : 
    extendYSub(n.subscribe("/hci/arm/gamepad/button/y", 
        MESSAGE_QUEUE_LENGTH, &SolenoidController::extendSolenoid, this)),
    yPressed(false) {}

void SolenoidController::extendSolenoid(const std_msgs::Bool::ConstPtr& msg)
{
    // This should extend the solenoid
    // ROS_INFO("I heard: [%s]", msg->data);
    this->yPressed = (msg->data != 0U);
}

void SolenoidController::checkMessage()
{
    while (ros::ok())
    {
        if (yPressed)
        {
            std_msgs::Int16 msgY;

            char buff = '1';
            int file = open("/sys/class/gpio/gpio6/value", "w");
            write (file, &buff, 1);
            close(file);
        }
        else
        {
            std_msgs::Int16 msgY;
            msgY.data = 0;
        }
    }
}