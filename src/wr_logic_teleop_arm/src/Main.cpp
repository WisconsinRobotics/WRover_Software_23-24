#include "ClawController.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "Main"); // TODO : May need more descriptive node name

    ros::NodeHandle nHand;
    ClawController claw = *new ClawController(nHand); // TODO : brace-initialization, can probably be stack allocated.
    // TODO : In either case: modernize - prefer smart pointers (in this case unique_pointer)
    

    return 0;
}