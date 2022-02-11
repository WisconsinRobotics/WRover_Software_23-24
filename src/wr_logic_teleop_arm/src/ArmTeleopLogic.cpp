#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "moveit_msgs/RobotTrajectory.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "boost/function.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <atomic>
#include <cstdint>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

using Std_Bool = const std_msgs::BoolConstPtr&;
using Std_Float32 = const std_msgs::Float32ConstPtr&;

std::atomic_bool isNewPath { true };
const tf2::Quaternion WORLD_OFFSET {0, sin(M_PI/2), 0, cos(M_PI/2)};

auto updateTarget(float x_pos, float y_pos, float z_pos, tf2::Quaternion orientation, ros::Publisher &pub) -> void{
    geometry_msgs::PoseStamped p {};
    p.pose.position.x = x_pos;
    p.pose.position.y = y_pos;
    p.pose.position.z = z_pos;
    auto outOrientation = /* tf2::Quaternion(0,sin(M_PI/4), 0, cos(M_PI/4)) * */ orientation;
    p.pose.orientation = tf2::toMsg(outOrientation);
    p.header.frame_id = "base_link";
    pub.publish(p);
    isNewPath.store(true);
}

auto main(int argc, char** argv) -> int{
    ros::init(argc, argv, "ArmTeleopLogic");
    ros::AsyncSpinner spin(1);
    spin.start();
    ros::NodeHandle np("~"); 

    constexpr float PLANNING_TIME = 0.05;
    constexpr float CLOCK_RATE = 2;
    constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000; 
    constexpr float TRIGGER_PRESSED = 0.5;
    constexpr float MINIMUM_PATH_ACCURACY = 0.0;

    constexpr float STEP_X = 0.003;
    constexpr float STEP_Y = 0.003;
    constexpr float STEP_Z = 0.003;

    constexpr float HOME_X = 0.7;
    constexpr float HOME_Y = 0.2;
    constexpr float HOME_Z = 0;

    float x_pos = HOME_X;
    float y_pos = HOME_Y;
    float z_pos = HOME_Z;

    tf2::Quaternion orientation {0, sin(-M_PI/4), 0, cos(-M_PI/4)};
    orientation = orientation;

    const tf2::Quaternion SPIN_X {sin(2*M_PI/1000), 0, 0, cos(2*M_PI/1000)};
    const tf2::Quaternion SPIN_Y {0, sin(2*M_PI/1000), 0, cos(2*M_PI/1000)};
    const tf2::Quaternion SPIN_Z {0, 0, sin(2*M_PI/1000), cos(2*M_PI/1000)};

    moveit::planning_interface::MoveGroupInterface move("arm");
    // move.setPlannerId("RRTStar");
    move.setPlanningTime(PLANNING_TIME);
    const moveit::core::JointModelGroup* joint_model_group = move.getCurrentState()->getJointModelGroup("arm");
    robot_state::RobotState start_state(*move.getCurrentState());
    moveit_visual_tools::MoveItVisualTools visual_tools("arm");
    std::cout << "interface handle: " << move.getNodeHandle().getNamespace() << std::endl;


    ros::Rate loop {CLOCK_RATE};

    ros::Publisher nextTarget = np.advertise<geometry_msgs::PoseStamped>("/logic/arm_teleop/next_target", 
        MESSAGE_QUEUE_LENGTH);
    
    ros::Publisher trajectoryPub = np.advertise<visualization_msgs::MarkerArray>("/logic/arm_teleop/trajectory", 
        MESSAGE_QUEUE_LENGTH);
        
    ros::Subscriber yAxis = np.subscribe("/xbox_test/axis/pov_y", 
        MESSAGE_QUEUE_LENGTH, 
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(msg->data != 0){
                y_pos += static_cast<bool>(msg->data) ? msg->data > 0 ? STEP_Y : -STEP_Y : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber xAxis = np.subscribe("/xbox_test/axis/pov_x", 
        MESSAGE_QUEUE_LENGTH, 
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(msg->data != 0){
                x_pos += static_cast<bool>(msg->data) ? msg->data > 0 ? STEP_X : -STEP_X : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber zUp = np.subscribe("/xbox_test/button/shoulder_l",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
            if(msg->data){
                z_pos += msg->data ? STEP_Z : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber zDown = np.subscribe("/xbox_test/axis/trigger_left",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(msg->data >= TRIGGER_PRESSED){
                z_pos += msg->data >= TRIGGER_PRESSED ? -STEP_Z : 0;
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));
    
    ros::Subscriber roll = np.subscribe("/xbox_test/axis/stick_left_x",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(abs(msg->data) >= 0.5){
                orientation *= (msg->data > 0 ? SPIN_Z : SPIN_Z.inverse());
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber pitch = np.subscribe("/xbox_test/axis/stick_left_y",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(abs(msg->data) >= 0.5){
                orientation *= (msg->data > 0 ? SPIN_Y : SPIN_Y.inverse());
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber yaw = np.subscribe("/xbox_test/axis/stick_right_x",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Float32)>>([&](Std_Float32 msg) -> void {
            if(abs(msg->data) >= 0.5){
                orientation *= (msg->data > 0 ? SPIN_X : SPIN_X.inverse());
                updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            }
        }));

    ros::Subscriber execPath = np.subscribe("/xbox_test/button/a",
        MESSAGE_QUEUE_LENGTH,
        static_cast<boost::function<void(Std_Bool)>>([&](Std_Bool msg) -> void {
            if(msg->data){
                isNewPath.store(true);
            }
        }));
        
    // transform = move.getCurrentState()->getFrameTransform("odom_combined");
    geometry_msgs::PoseStamped endEffectorPose = move.getCurrentPose();
    x_pos = endEffectorPose.pose.position.x;
    y_pos = endEffectorPose.pose.position.x;
    z_pos = endEffectorPose.pose.position.x;
    x_pos = endEffectorPose.pose.position.x;

    orientation = tf2::Quaternion(
        endEffectorPose.pose.orientation.x,
        endEffectorPose.pose.orientation.y,
        endEffectorPose.pose.orientation.z,
        endEffectorPose.pose.orientation.w
    );

    updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);

    while(ros::ok()){

        if(!isNewPath.load()){
            // updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            loop.sleep();
            continue;
        }

        // stop current path
        isNewPath.store(false);
        move.stop();

        // configure target pose
        geometry_msgs::PoseStamped p {};
        p.pose.position.x = x_pos;
        p.pose.position.y = y_pos;
        p.pose.position.z = z_pos;
        p.pose.orientation = tf2::toMsg(orientation);
        p.header.frame_id = "odom_combined";

        move.setPoseTarget(p);
        move.setStartStateToCurrentState();

        //plan and execute path
        move.asyncMove();

        while(ros::ok){
            // updateTarget(x_pos, y_pos, z_pos, orientation, nextTarget);
            // std::cout << "[INFO] [" << ros::Time::now() << "]: executing path " << std::endl;

            if(move.getMoveGroupClient().getState().isDone()){
                std::cout << "[INFO] [" << ros::Time::now() << "]: path finished: " << move.getMoveGroupClient().getState().getText() << std::endl;
                break;
            } 
            
            else if(isNewPath.load()){
                std::cout << "[INFO] [" << ros::Time::now() << "]: path overridden" << std::endl;
                move.stop();
                break;
            }

            loop.sleep();
        }   
    }

    return 0;
}