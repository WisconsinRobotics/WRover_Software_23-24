#include "SolenoidController.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

#include <fcntl.h>
#include <fstream>



constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;
constexpr uint16_t pin = 6;
std::ofstream file;
std::ofstream fileExport;


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

    // TODO ; Make sure you set up the GPIO pin in the constructor!
}

void SolenoidController::extendSolenoid(const std_msgs::Bool::ConstPtr& msg)
{
    // This should extend the solenoid
    this->yPressed = (msg->data != 0U);

    if (yPressed)
    {


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

SolenoidController::~SolenoidController()
{
    // TODO : (And take down the pin in the destructor "~SolenoidController()")
}