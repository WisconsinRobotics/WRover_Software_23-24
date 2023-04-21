#include "ClawController.hpp"
#include "SolenoidController.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"

auto main(int argc, char **argv) -> int
{
    ros::init(argc, argv, "ClawAndSolenoidNode"); // TODO : May need more descriptive node name

    ros::NodeHandle nHandCC;
    std::unique_ptr<ClawController> claw = new ClawController{nHandCC};
    
    ros::NodeHandle nHandSC;
    std::unique_ptr<SolenoidController> solenoid = new ClawController{nHandSC};

    return 0;
}