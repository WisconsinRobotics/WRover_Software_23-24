#include "ClawController.hpp"
#include "SolenoidController.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "ClawAndSolenoidNode");

    ros::NodeHandle nHand;
    ClawController claw {nHand};
    SolenoidController solenoid {nHand};

    ros::spin();

    return 0;
}