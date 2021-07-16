#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "boost/function.hpp"
#include <cstdint>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

using Std_Bool = const std_msgs::BoolConstPtr&;
using Std_Float32 = const std_msgs::Float32ConstPtr&;

auto main(int argc, char** argv) -> int{
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~"); 

    constexpr float PLANNING_TIME = 0.05;
    constexpr float CLOCK_RATE = 50;
    constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000; 
    constexpr float TRIGGER_PRESSED = 0.5;

    constexpr float STEP_X = 0.001;
    constexpr float STEP_Y = 0.001;
    constexpr float STEP_Z = 0.001;

    constexpr float HOME_X = 0.25;
    constexpr float HOME_Y = 0;
    constexpr float HOME_Z = 0.25;

    float x_pos = HOME_X;
    float y_pos = HOME_Y;
    float z_pos = HOME_Z;

    moveit::planning_interface::MoveGroupInterface move("arm");
    // move.setPlannerId("RRTStar");
    move.setPlanningTime(PLANNING_TIME);
    ros::Rate loop {CLOCK_RATE};

    ros::Subscriber yAxis = np.subscribe("/xbox_test/axis/pov_y", 
        MESSAGE_QUEUE_LENGTH, 
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {y_pos += static_cast<bool>(msg->data) ? msg->data > 0 ? STEP_Y : -STEP_Y : 0;}));
    ros::Subscriber xAxis = np.subscribe("/xbox_test/axis/pov_x", 
        MESSAGE_QUEUE_LENGTH, 
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {x_pos += static_cast<bool>(msg->data) ? msg->data > 0 ? STEP_X : -STEP_X : 0;}));
    ros::Subscriber zUp = np.subscribe("/xbox_test/button/shoulder_l",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {z_pos += msg->data ? STEP_Z : 0;}));
    ros::Subscriber zDown = np.subscribe("/xbox_test/axis/trigger_l",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {z_pos += msg->data > TRIGGER_PRESSED ? -STEP_Z : 0;}));

    while(ros::ok()){
        geometry_msgs::PoseStamped p {};
        p.pose.position.x = x_pos;
        p.pose.position.y = y_pos;
        p.pose.position.z = z_pos;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = sin(-M_PI/4);
        p.pose.orientation.z = 0;
        p.pose.orientation.w = cos(-M_PI/4);
        p.header.frame_id = "odom_combined";
        move.setPoseTarget(p);
        move.move();
        loop.sleep();
    }

    return 0;
}