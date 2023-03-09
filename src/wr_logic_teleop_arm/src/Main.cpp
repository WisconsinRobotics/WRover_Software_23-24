#include "ClawController.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "Main");

    ros::NodeHandle nHand;
    ClawController claw = *new ClawController(nHand);
    

    return 0;
}