/**
 * @file JointStatePublisher.cpp
 * @author Ben Nowotny
 * @author Jack Zautner
 * @brief The executable file to run the joint state publisher
 * @date 2022-12-05
 */

#include "SingleEncoderJointPositionMonitor.hpp"
#include "XmlRpcValue.h"

#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <array>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <csignal>
#include <memory>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <string>

using XmlRpc::XmlRpcValue;

/**
 * @brief Nessage cache size of publisher
 */
constexpr std::uint32_t MESSAGE_CACHE_SIZE = 1;

/**
 * @brief Period between timer callback
 */
constexpr float TIMER_CALLBACK_DURATION = 1.0 / 50.0;

/**
 * @brief The joint state publisher for MoveIt
 */
ros::Publisher jointStatePublisher;

std::map<std::string, SingleEncoderJointPositionMonitor> namedJointPositionMonitors;

void publishJointStates(const ros::TimerEvent &event) {
    std::vector<std::string> names;
    std::vector<double> positions;
    sensor_msgs::JointState js_msg;

    for (auto &[name, monitor] : namedJointPositionMonitors) {
        names.push_back(name);
        positions.push_back(monitor());
    }

    js_msg.name = names;
    js_msg.position = positions;
    js_msg.header.stamp = ros::Time::now();
    // Publish the Joint State message
    jointStatePublisher.publish(js_msg);
}

auto getEncoderConfigFromParams(const XmlRpcValue &params, const std::string &jointName) -> EncoderConfiguration {
    return {.countsPerRotation = static_cast<uint32_t>(static_cast<int32_t>(params[jointName]["counts_per_rotation"])),
            .offset = static_cast<uint32_t>(static_cast<int32_t>(params[jointName]["offset"]))};
}

/**
 * @brief The main executable method of the node. Starts the ROS node
 *
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
auto main(int argc, char **argv) -> int {

    std::cout << "start main" << std::endl;
    // // Initialize the current node as JointStatePublisherApplication
    ros::init(argc, argv, "JointStatePublisher");
    // // Create the NodeHandle to the current ROS node
    ros::NodeHandle n;
    ros::NodeHandle pn{"~"};

    XmlRpcValue encParams;
    pn.getParam("encoder_parameters", encParams);

    namedJointPositionMonitors.try_emplace("elbowPitch_joint",

                                           "aux1",
                                           RoboclawChannel::A,
                                           getEncoderConfigFromParams(encParams, "elbow"),
                                           n);
    namedJointPositionMonitors.try_emplace("elbowRoll_joint",

                                           "aux1",
                                           RoboclawChannel::B,
                                           getEncoderConfigFromParams(encParams, "forearmRoll"),
                                           n);
    namedJointPositionMonitors.try_emplace("shoulder_joint",

                                           "aux0",
                                           RoboclawChannel::B,
                                           getEncoderConfigFromParams(encParams, "shoulder"),
                                           n);
    namedJointPositionMonitors.try_emplace("turntable_joint",

                                           "aux0",
                                           RoboclawChannel::A,
                                           getEncoderConfigFromParams(encParams, "turntable"),
                                           n);
    namedJointPositionMonitors.try_emplace("wristPitch_joint",

                                           "aux2",
                                           RoboclawChannel::A,
                                           getEncoderConfigFromParams(encParams, "wristPitch"),
                                           n);
    namedJointPositionMonitors.try_emplace("wristRoll_link",
                                           "aux2",
                                           RoboclawChannel::B,
                                           getEncoderConfigFromParams(encParams, "wristRoll"),
                                           n);

    // Initialize the Joint State Data Publisher
    jointStatePublisher = n.advertise<sensor_msgs::JointState>("/joint_states", MESSAGE_CACHE_SIZE);

    // Timer that will call publishJointStates periodically
    ros::Timer timer = n.createTimer(ros::Duration(TIMER_CALLBACK_DURATION), publishJointStates);

    // Enter ROS spin
    ros::spin();

    return 0;
}