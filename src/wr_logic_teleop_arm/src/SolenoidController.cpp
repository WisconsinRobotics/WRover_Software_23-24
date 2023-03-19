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

// TODO ; Make sure you set up the GPIO pin in the constructor!
// TODO : (And take down the pin in the destructor "~SolenoidController()")
SolenoidController::SolenoidController(ros::NodeHandle& n) : 
    extendYSub(n.subscribe("/hci/arm/gamepad/button/y", 
        MESSAGE_QUEUE_LENGTH, &SolenoidController::extendSolenoid, this)),
    yPressed(false) {} // TODO : Use brace-initialization

void SolenoidController::extendSolenoid(const std_msgs::Bool::ConstPtr& msg)
{
    // This should extend the solenoid
    // ROS_INFO("I heard: [%s]", msg->data);
    this->yPressed = (msg->data != 0U);
}

void SolenoidController::checkMessage()
{
    while (ros::ok()) // TODO : ROS-OK check not needed here
    {
        if (yPressed)
        {
            std_msgs::Int16 msgY; // TODO : This behavior can move to the extendSolenoid function, and then you may not need this method at all.

            char buff = '1';  // TODO : Where do we close?
            int file = open("/sys/class/gpio/gpio6/value", "w"); // TODO : Modernize - prefer std::(o)fstream for RAII handling
            write (file, &buff, 1);
            close(file); // TODO : Maybe open in the constructor to avoid constantly re-opening the same file, replace with flush
        }
        else
        {
            std_msgs::Int16 msgY;
            msgY.data = 0;
        }
    }
}