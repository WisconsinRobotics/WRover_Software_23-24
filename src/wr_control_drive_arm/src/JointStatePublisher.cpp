/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file JointStatePublisher.cpp
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

void publishJointStates(const std::unordered_map<std::string, const std::shared_ptr<const SingleEncoderJointPositionMonitor>> &namedJointPositionMonitors,
                        const ros::Publisher &jointStatePublisher) {
    std::vector<std::string> names;
    std::vector<double> positions;
    sensor_msgs::JointState js_msg;

    for (const auto &[name, monitor] : namedJointPositionMonitors) {
        names.push_back(name);
        positions.push_back((*monitor)());
    }

    js_msg.name = names;
    js_msg.position = positions;
    js_msg.header.stamp = ros::Time::now();
    // Publish the Joint State message
    jointStatePublisher.publish(js_msg);
}

/**
 * @brief Get the Encoder Config from ROS Params
 *
 * @param params The dictionary of ROS parameters
 * @param jointName The name of the joint requested
 * @return EncoderConfiguration The encoder configuration of that joint
 */
auto getEncoderConfigFromParams(const XmlRpcValue &params, const std::string &jointName) -> EncoderConfiguration {
    return {.countsPerRotation = static_cast<int32_t>(params[jointName]["encoder_parameters"]["counts_per_rotation"]),
            .offset = static_cast<int32_t>(params[jointName]["encoder_parameters"]["offset"])};
}

/**
 * @brief Get the Motor Config from ROS Params
 *
 * @param params The dictionary of ROS parameters
 * @param jointName The name of the joint requested
 * @return MotorConfiguration The motor configuration of that joint
 */
auto getMotorConfigFromParams(const XmlRpcValue &params, const std::string &jointName) -> MotorConfiguration {
    return {.gearRatio = static_cast<double>(params[jointName]["motor_configurations"]["gear_ratio"])};
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
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle{"~"};

    XmlRpcValue armParams;
    privateNodeHandle.getParam("arm_parameters", armParams);

    const std::unordered_map<std::string, const std::shared_ptr<const SingleEncoderJointPositionMonitor>> namedPositionMonitors{
        {"turntable_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                "aux0",
                                RoboclawChannel::A,
                                getEncoderConfigFromParams(armParams, "turntable"),
                                getMotorConfigFromParams(armParams, "turntable"),
                                nodeHandle)},

        {"shoulder_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                               "aux0",
                               RoboclawChannel::B,
                               getEncoderConfigFromParams(armParams, "shoulder"),
                               getMotorConfigFromParams(armParams, "shoulder"),
                               nodeHandle)},

        {"elbowPitch_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                 "aux1",
                                 RoboclawChannel::A,
                                 getEncoderConfigFromParams(armParams, "elbow"),
                                 getMotorConfigFromParams(armParams, "elbow"),
                                 nodeHandle)},

        {"elbowRoll_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                "aux1",
                                RoboclawChannel::B,
                                getEncoderConfigFromParams(armParams, "forearmRoll"),
                                getMotorConfigFromParams(armParams, "forearmRoll"),
                                nodeHandle)},

        {"wristPitch_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                 "aux2",
                                 RoboclawChannel::A,
                                 getEncoderConfigFromParams(armParams, "wristPitch"),
                                 getMotorConfigFromParams(armParams, "wristPitch"),
                                 nodeHandle)},

        {"wristRoll_link", std::make_shared<SingleEncoderJointPositionMonitor>(
                               "aux2",
                               RoboclawChannel::B,
                               getEncoderConfigFromParams(armParams, "wristRoll"),
                               getMotorConfigFromParams(armParams, "wristRoll"),
                               nodeHandle)}};

    // Initialize the Joint State Data Publisher
    ros::Publisher jointStatePublisher{nodeHandle.advertise<sensor_msgs::JointState>("/joint_states", MESSAGE_CACHE_SIZE)};

    // Timer that will call publishJointStates periodically
    ros::Timer timer = nodeHandle.createTimer(ros::Duration(TIMER_CALLBACK_DURATION),
                                              [&namedPositionMonitors, &jointStatePublisher](const ros::TimerEvent &) {
                                                  publishJointStates(namedPositionMonitors, jointStatePublisher);
                                              });

    // Enter ROS spin
    ros::spin();

    return 0;
}

/// @}
