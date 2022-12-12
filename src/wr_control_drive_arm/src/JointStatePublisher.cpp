/**
 * @file JointStatePublisher.cpp
 * @author Ben Nowotny
 * @author Jack Zautner
 * @brief The executable file to run the joint state publisher
 * @date 2022-12-05 
 */
#include "XmlRpcValue.h"

#include "moveit_msgs/DisplayRobotState.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <algorithm>
#include <csignal>
#include <string>
#include <array>
#include <memory>
#include <std_srvs/Trigger.h>
#include "SimpleJoint.hpp"
#include "DifferentialJoint.hpp"
#include "ros/console.h"

using XmlRpc::XmlRpcValue;

/**
 * @brief Nessage cache size of publisher
 */
constexpr std::uint32_t MESSAGE_CACHE_SIZE = 1000; 

/**
 * @brief Period between timer callback
 */
constexpr float TIMER_CALLBACK_DURATION = 1.0 / 50.0;

/**
 * @brief Defines space for all Joint references
 */
constexpr int NUM_JOINTS = 5;
std::array<std::unique_ptr<AbstractJoint>, NUM_JOINTS> joints;

/**
 * @brief The joint state publisher for MoveIt
 */
ros::Publisher jointStatePublisher;

void publishJointStates(const ros::TimerEvent &event){
  std::vector<std::string> names;
  std::vector<double> positions;
  sensor_msgs::JointState js_msg;

  for(const auto &joint : joints){
    for(int i = 0; i < joint->getDegreesOfFreedom(); i++){
      names.push_back(joint->getMotor(i)->getMotorName());
      positions.push_back(joint->getMotor(i)->getRads());
    }
  }

  js_msg.name = names;
  js_msg.position = positions;
  js_msg.header.stamp = ros::Time::now();
  
  // Publish the Joint State message and robot state message
  jointStatePublisher.publish(js_msg);
}

/**
 * @brief The main executable method of the node. Starts the ROS node
 * 
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
auto main(int argc, char** argv) -> int {

    std::cout << "start main" << std::endl;
    // Initialize the current node as JointStatePublisherApplication
    ros::init(argc, argv, "JointStatePublisher");
    // Create the NodeHandle to the current ROS node
    ros::NodeHandle n;
    ros::NodeHandle pn{"~"};

    XmlRpcValue encParams;
    pn.getParam("encoder_parameters", encParams);

    // Initialize all motors with their MoveIt name, WRoboclaw initialization, and reference to the current node
    auto elbowPitch_joint = std::make_unique<ArmMotor>("elbowPitch_joint", 1, 0, static_cast<int>(encParams[0]["counts_per_rotation"]), static_cast<int>(encParams[0]["offset"]), n);
    auto elbowRoll_joint = std::make_unique<ArmMotor>("elbowRoll_joint", 1, 1, static_cast<int>(encParams[1]["counts_per_rotation"]), static_cast<int>(encParams[1]["offset"]), n);
    auto shoulder_joint = std::make_unique<ArmMotor>("shoulder_joint", 0, 1, static_cast<int>(encParams[2]["counts_per_rotation"]), static_cast<int>(encParams[2]["offset"]), n);
    auto turntable_joint = std::make_unique<ArmMotor>("turntable_joint", 0, 0, static_cast<int>(encParams[3]["counts_per_rotation"]), static_cast<int>(encParams[3]["offset"]), n);
    auto wristPitch_joint = std::make_unique<ArmMotor>("wristPitch_joint", 2, 0, static_cast<int>(encParams[4]["counts_per_rotation"]), static_cast<int>(encParams[4]["offset"]), n);
    auto wristRoll_link = std::make_unique<ArmMotor>("wristRoll_link", 2, 1, static_cast<int>(encParams[5]["counts_per_rotation"]), static_cast<int>(encParams[5]["offset"]), n);
    std::cout << "init motors" << std::endl;

    // Initialize all Joints
    joints.at(0) = std::make_unique<SimpleJoint>(std::move(elbowPitch_joint), n);
    joints.at(1) = std::make_unique<SimpleJoint>(std::move(elbowRoll_joint), n);
    joints.at(2) = std::make_unique<SimpleJoint>(std::move(shoulder_joint), n);
    joints.at(3) = std::make_unique<SimpleJoint>(std::move(turntable_joint), n);
    joints.at(4) = std::make_unique<DifferentialJoint>(std::move(wristPitch_joint), std::move(wristRoll_link), n, "/control/arm/5/pitch", "/control/arm/5/roll", "/control/arm/20/", "/control/arm/21/");
    std::cout << "init joints" << std::endl;
  
    // Initialize the Joint State Data Publisher
    jointStatePublisher = n.advertise<sensor_msgs::JointState>("/joint_states", MESSAGE_CACHE_SIZE);

    // Timer that will call publishJointStates periodically
    ros::Timer timer = n.createTimer(ros::Duration(TIMER_CALLBACK_DURATION), publishJointStates);

    // Enter ROS spin
    ros::spin();

    moveit_msgs::DisplayRobotState joe;
    joe.state.joint_state;

    return 0;
}