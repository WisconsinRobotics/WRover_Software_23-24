#include <ros/ros.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~");

    moveit::planning_interface::MoveGroupInterface move("arm");
    // move.setPlannerId("RRTStar");
    move.setPlanningTime(0.15);
    double x = -0.5;
    int i = 0;
    while(x<0.5){
        geometry_msgs::PoseStamped p;
        p.pose.position.x = 0.25;
        p.pose.position.y = x;
        p.pose.position.z = 0.25;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = sin(-M_PI/4);
        p.pose.orientation.z = 0;
        p.pose.orientation.w = cos(-M_PI/4);
        p.header.frame_id = "odom_combined";
        move.setPoseTarget(p);
        move.move();
        x+=0.05;
        i++;
    }

    return 0;
}