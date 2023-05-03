#include "SolenoidController.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

#include <fcntl.h>
#include <fstream>

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

    fileUnexport.open("/sys/class/gpio/unexport");
    if (!fileUnexport.is_open())
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
        file << 1;
        file.flush();
    }
}

SolenoidController::~SolenoidController()
{
    fileUnexport << pin;
    fileUnexport.flush();
}