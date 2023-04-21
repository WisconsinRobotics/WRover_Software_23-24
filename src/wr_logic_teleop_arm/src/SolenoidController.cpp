#include "SolenoidController.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"
#include <fcntl.h>
#include <fstream>

// Again, you may want a constructor here
// For pass-by-value reasons, you may want to take the NodeHandle as an argument
// And construct the Publisher in this constructor, rather that taking the Publisher as 
// an argument.

constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;
constexpr uint16_t pin = 6;
std::ofstream file;
std::ofstream fileExport;

// TODO ; Make sure you set up the GPIO pin in the constructor!
// TODO : (And take down the pin in the destructor "~SolenoidController()")
SolenoidController::SolenoidController(ros::NodeHandle& n) : 
    extendYSub {n.subscribe("/hci/arm/gamepad/button/y", MESSAGE_QUEUE_LENGTH, 
        &SolenoidController::extendSolenoid, this)}, 
    yPressed {false}
{
    file.open("/sys/class/gpio/gpio6/value");
    if (!file.is_open())
    {
        std::cout << "Unable to open /sys/class/gpio/gpio6/value";
    }

    fileExport.open("/sys/class/gpio/unexport");
    if (!fileExport.is_open())
    {
        std::cout << "Unable to open /sys/class/gpio/unexport";
    }
}

void SolenoidController::extendSolenoid(const std_msgs::Bool::ConstPtr& msg)
{
    // This should extend the solenoid
    this->yPressed = (msg->data != 0U);

    if (yPressed)
    {
        std_msgs::Int16 msgY; // TODO : This behavior can move to the extendSolenoid function, and then you may not need this method at all.

        file << 1;
        file.flush();
            
        fileExport << pin;
        fileExport.flush();
    }
    else
    {
        file << 0;
        file.flush();

        fileExport << pin;
        fileExport.flush();
    }
}