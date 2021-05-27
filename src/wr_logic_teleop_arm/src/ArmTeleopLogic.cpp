#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~");

    moveit::planning_interface::MoveGroupInterface move("arm");
    // move.setPlannerId("RRTStar");
    move.setPlanningTime(0.5);
    move.setPositionTarget(0.35, 0.35, 0.35);
    move.move();

    return 0;
}