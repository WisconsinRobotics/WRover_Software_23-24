#include "SolenoidController.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

#include <fcntl.h>
#include <fstream>
#include <chrono>
#include <thread>

SolenoidController::SolenoidController(ros::NodeHandle& n) : 
    extendYSub {n.subscribe("/hci/arm/gamepad/button/y", MESSAGE_QUEUE_LENGTH, 
        &SolenoidController::extendSolenoid, this)}, 
    yPressed {false}
{
    fileExport.open("/sys/class/gpio/export");
    if (!fileExport.is_open()) {
        std::cout << "Unable to open /sys/class/gpio/export";
    }
    fileExport << pin;

    while (access("/sys/class/gpio/gpio6/value", F_OK) != 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }
    
    file.open("/sys/class/gpio/gpio6/value");
    if (!file.is_open()) {
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
    else 
    {
        file << 0;
        file.flush();
    }
}

SolenoidController::~SolenoidController()
{
    file << 0;
    fileUnexport << pin;
}