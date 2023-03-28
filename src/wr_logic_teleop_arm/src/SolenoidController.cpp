#include "SolenoidController.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include <fcntl.h>

// Again, you may want a constructor here
// For pass-by-value reasons, you may want to take the NodeHandle as an argument
// And construct the Publisher in this constructor, rather that taking the Publisher as 
// an argument.

constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;
constexpr uint16_t pin = 6;

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
        if (yPressed) {
            char buff = '1';
            int file = open("/sys/class/gpio/gpio6/value", O_WRONLY);
            if (file == -1)
            {
                perror("Unable to open /sys/class/gpio/gpio6/value");
            }
            write (file, &buff, 1);
            close(file);

            int fileExport = open("/sys/class/gpio/unexport", O_WRONLY);
            if (fileExport == -1)
            {
                perror("Unable to open /sys/class/gpio/unexport");
            }
            write (fileExport, &buff, pin);
            close(fileExport);
        }
        else
        {
            char buff = '1';
            int file = open("/sys/class/gpio/gpio6/value", O_WRONLY);
            if (file == -1)
            {
                perror("Unable to open /sys/class/gpio/gpio6/value");
            }
            write (file, &buff, 0);
            close(file);

            int fileExport = open("/sys/class/gpio/unexport", O_WRONLY);
            if (fileExport == -1)
            {
                perror("Unable to open /sys/class/gpio/unexport");
            }
            write (fileExport, &buff, pin);
            close(fileExport);
        }
    }
}