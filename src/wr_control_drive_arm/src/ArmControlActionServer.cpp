/**
 * @ingroup wr_control_drive_arm
 * @{
 */

/**
 * @file ArmControlActionServer.cpp
 * @author Ben Nowotny
 * @brief The exeutable file to run the Arm Control Action Server
 * @date 2022-12-05
 */

#include "DifferentialJointToMotorSpeedConverter.hpp"
#include "DirectJointToMotorSpeedConverter.hpp"
#include "Joint.hpp"
#include "Motor.hpp"
#include "RoboclawChannel.hpp"
#include "SingleEncoderJointPositionMonitor.hpp"
#include "XmlRpcValue.h"
#include "ros/init.h"

#include <actionlib/server/simple_action_server.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <csignal>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using XmlRpc::XmlRpcValue;

/// Tick rate of the action server
constexpr float CLOCK_RATE_HZ{50};

/// Delay when publishing IK warnings
constexpr double IK_WARN_PERIOD_SEC{1.0 / 2};

/// Max speed for a joint (motor power) when moving between setpoints
constexpr double JOINT_SAFETY_MAX_SPEED{0.5};
/// Max speed for a joint (motor power) when it has reached a setpoint
constexpr double JOINT_SAFETY_HOLD_SPEED{0.3};

/// Message cache size of publisher
constexpr std::uint32_t MESSAGE_CACHE_SIZE{10};

/// Period between timer callbacks when checking overcurrent errors
constexpr float OVERCURRENT_TIMEOUT_PERIOD_SEC{1.0 / 50.0};

/// Map of joint names to @ref Joint objects
std::unordered_map<std::string, std::unique_ptr<Joint>> namedJointMap;

/// Map of joint names to position monitors
std::unordered_map<std::string, std::shared_ptr<SingleEncoderJointPositionMonitor>> namedPositionMonitors;

/// Simplify actionserver namespace/templatization
using Server = actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>;

/// Service to enable IK if it has been disabled for some reason
ros::ServiceServer enableServiceServer;

/// Enabled status of the IK program
std::atomic_bool IKEnabled{true};

/**
 * @brief Perform the given action as interpreted as moving the arm joints to
 * specified positions
 *
 * @param goal The goal state given
 * @param as The Action Server this is occuring on
 */
void execute(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal,
             Server *server) {

    // If IK is disabled, don't do anything and report status
    if (!IKEnabled) {
        server->setAborted();
        ROS_WARN_THROTTLE( // NOLINT; ROS macro excuse
            IK_WARN_PERIOD_SEC,
            "IK is disabled");
        return;
    }

    // DEBUGGING, not required to execute trajectory
    for (const auto &jointName : goal->trajectory.joint_names) {
        std::cout << jointName << "\t";
    }
    std::cout << std::endl;
    for (const auto &waypoint : goal->trajectory.points) {
        for (const auto &jointVal : waypoint.positions) {
            std::cout << jointVal << "\t";
        }
        std::cout << std::endl;
    }

    // The path is broken up into smaller waypoints serving as intermediate targets
    // For each waypoint in the path...
    for (const auto &currTargetPosition : goal->trajectory.points) {

        // If IK is disabled during the program, report fault and exit
        if (!IKEnabled) {
            server->setAborted();
            std::cout << "Over current fault!" << std::endl;
            return;
        }

        std::vector<double> velocityCopies{currTargetPosition.velocities};

        // Scale joint speeds by gear ratio of the joint
        for (uint32_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {

            // The joint that is currently being scaled
            auto currJoint = goal->trajectory.joint_names.at(i);

            // The position monitor whose velocity is currently being scaled
            auto currPosMtr = namedPositionMonitors.at(currJoint);

            // Scale by counts per rotation and gear ratio
            velocityCopies.at(i) *= abs(currPosMtr->getCountsPerRotation()*currPosMtr->getGearRatio());
        }

        // Get the maximum velocity assigned to any joint
        const double VELOCITY_MAX = abs(*std::max_element(
            velocityCopies.begin(),
            velocityCopies.end(),
            [](double lhs, double rhs) -> bool { return abs(lhs) < abs(rhs); }));

        // Scale joint speeds so that none is greater than JOINT_SAFETY_MAX_SPEED
        for (uint32_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
            // Set joint to hold speed in case the greatest velocity comes through as 0
            auto jointVelocity{JOINT_SAFETY_HOLD_SPEED};

            // Scale all velocities by the safety max speed with respect to the maximum velocity given by MoveIt
            if (VELOCITY_MAX != 0)
                jointVelocity = velocityCopies.at(i) / VELOCITY_MAX * JOINT_SAFETY_MAX_SPEED;

            // Set the joint's velocity
            namedJointMap.at(goal->trajectory.joint_names.at(i))->setTarget(currTargetPosition.positions.at(i), jointVelocity);
        }
    }

    auto waypointComplete{false};
    ros::Rate updateRate{CLOCK_RATE_HZ};

    // While...
    // The waypoint is not complete
    // ROS has not stopped, and
    // There isn't a newer target available
    while (!waypointComplete && ros::ok() && !server->isNewGoalAvailable()) {

        // If IK is disabled during the program, report fault and exit
        if (!IKEnabled) {
            server->setAborted();
            std::cout << "Over current fault!" << std::endl;
            return;
        }

        // A waypoint is complete if every joint has reached its target
        waypointComplete = true;
        for (const auto &[_, joint] : namedJointMap) {
            waypointComplete &= joint->hasReachedTarget();
        }

        // Wait to update
        updateRate.sleep();
    }

    // Report preemption if it occurred
    if (server->isNewGoalAvailable())
        server->setPreempted();
    // When all positions have been reached, set the current task as succeeded
    else
        server->setSucceeded();
    std::cout << "Action Complete!" << std::endl;
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
 * @brief ROS timed callback to check if motors are overcurrent.  
 * IK is disabled if an overcurrent fault happens
 * 
 * @param motors The list of motors to check
 */
void checkOverCurrentFaults(const std::vector<std::shared_ptr<Motor>> &motors){
    for(const auto& motor : motors){
        if (motor->isOverCurrent()) {
            IKEnabled = false;
            std::cout << "Over current fault!" << std::endl;

            for (const auto& joint : namedJointMap) {
                joint.second->stop();
            }
        }
        // TODO:  arbitrate control to old arm driver
    }
}

/**
 * @brief The main executable method of the node.  Starts the ROS node and the
 * Action Server for processing Arm Control commands
 *
 * @param argc The number of program arguments
 * @param argv The given program arguments
 * @return int The status code on exiting the program
 */
auto main(int argc, char **argv) -> int {
    std::cout << "start main" << std::endl;
    // Initialize the current node as ArmControlSystem
    ros::init(argc, argv, "ArmControlActionServer");
    // Create the NodeHandle to the current ROS node
    ros::NodeHandle n;
    ros::NodeHandle pn{"~"};

    XmlRpcValue armParams;
    pn.getParam("arm_parameters", armParams);

    // Initialize all motors with their MoveIt name, WRoboclaw initialization,
    // and reference to the current node
    std::cout << "init motors" << std::endl;

    // Setup the list of motors
    std::vector<std::shared_ptr<Motor>> motors{};

    using std::literals::string_literals::operator""s;

    const auto turntableMotor{std::make_shared<Motor>("aux0"s, RoboclawChannel::A, n)};
    const auto shoulderMotor{std::make_shared<Motor>("aux0"s, RoboclawChannel::B, n)};
    const auto elbowMotor{std::make_shared<Motor>("aux1"s, RoboclawChannel::A, n)};
    const auto forearmRollMotor{std::make_shared<Motor>("aux1"s, RoboclawChannel::B, n)};
    const auto wristLeftMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::A, n)};
    const auto wristRightMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::B, n)};

    motors.push_back(turntableMotor);
    motors.push_back(shoulderMotor);
    motors.push_back(elbowMotor);
    motors.push_back(forearmRollMotor);
    motors.push_back(wristLeftMotor);
    motors.push_back(wristRightMotor);

    // Create/name the encoders
    namedPositionMonitors.insert({"turntable_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                                            "aux0"s,
                                                            RoboclawChannel::A,
                                                            getEncoderConfigFromParams(armParams, "turntable"),
                                                            getMotorConfigFromParams(armParams, "turntable"),
                                                            n)});

    namedPositionMonitors.insert({"shoulder_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                                            "aux0"s,
                                                            RoboclawChannel::B,
                                                            getEncoderConfigFromParams(armParams, "shoulder"),
                                                            getMotorConfigFromParams(armParams, "shoulder"),
                                                            n)});

    namedPositionMonitors.insert({"elbowPitch_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                                            "aux1"s,
                                                            RoboclawChannel::A,
                                                            getEncoderConfigFromParams(armParams, "elbow"),
                                                            getMotorConfigFromParams(armParams, "elbow"),
                                                            n)});

    namedPositionMonitors.insert({"elbowRoll_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                                            "aux1"s,
                                                            RoboclawChannel::B,
                                                            getEncoderConfigFromParams(armParams, "forearmRoll"),
                                                            getMotorConfigFromParams(armParams, "forearmRoll"),
                                                            n)});

    namedPositionMonitors.insert({"wristPitch_joint", std::make_shared<SingleEncoderJointPositionMonitor>(
                                                            "aux2"s,
                                                            RoboclawChannel::A,
                                                            getEncoderConfigFromParams(armParams, "wristPitch"),
                                                            getMotorConfigFromParams(armParams, "wristPitch"),
                                                            n)});

    namedPositionMonitors.insert({"wristRoll_link", std::make_shared<SingleEncoderJointPositionMonitor>(
                                                            "aux2"s,
                                                            RoboclawChannel::B,
                                                            getEncoderConfigFromParams(armParams, "wristRoll"),
                                                            getMotorConfigFromParams(armParams, "wristRoll"),
                                                            n)});

    // Create the speed converters

    const auto turntableSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(turntableMotor, MotorSpeedDirection::REVERSE)};
    const auto shoulderSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(shoulderMotor, MotorSpeedDirection::REVERSE)};
    const auto elbowSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(elbowMotor, MotorSpeedDirection::REVERSE)};
    const auto forearmRollSpeedConverter{std::make_shared<DirectJointToMotorSpeedConverter>(forearmRollMotor, MotorSpeedDirection::REVERSE)};
    const auto differentialSpeedConverter{std::make_shared<DifferentialJointToMotorSpeedConverter>(wristLeftMotor, wristRightMotor)};

    // Initialize/name all the Joints

    std::cout << "init joints" << std::endl;

    namedJointMap.insert({"turntable_joint", std::make_unique<Joint>(
                                                 "turntable"s,
                                                 [turntablePositionMonitor=namedPositionMonitors.at("turntable_joint")]() -> double { return (*turntablePositionMonitor)(); },
                                                 [turntableSpeedConverter](double speed) { (*turntableSpeedConverter)(speed); },
                                                 n)});
    namedJointMap.insert({"shoulder_joint", std::make_unique<Joint>(
                                                "shoulder",
                                                [shoulderPositionMonitor=namedPositionMonitors.at("shoulder_joint")]() -> double { return (*shoulderPositionMonitor)(); },
                                                [shoulderSpeedConverter](double speed) { (*shoulderSpeedConverter)(speed); },
                                                n)});
    namedJointMap.insert({"elbowPitch_joint", std::make_unique<Joint>(
                                                  "elbow",
                                                  [elbowPositionMonitor=namedPositionMonitors.at("elbowPitch_joint")]() -> double { return (*elbowPositionMonitor)(); },
                                                  [elbowSpeedConverter](double speed) { (*elbowSpeedConverter)(speed); },
                                                  n)});
    namedJointMap.insert({"elbowRoll_joint", std::make_unique<Joint>(
                                                 "forearmRoll",
                                                 [forearmRollPositionMonitor=namedPositionMonitors.at("elbowRoll_joint")]() -> double { return (*forearmRollPositionMonitor)(); },
                                                 [forearmRollSpeedConverter](double speed) { (*forearmRollSpeedConverter)(speed); },
                                                 n)});
    namedJointMap.insert({"wristPitch_joint", std::make_unique<Joint>(
                                                  "wristPitch",
                                                  [wristPitchPositionMonitor=namedPositionMonitors.at("wristPitch_joint")]() -> double { return (*wristPitchPositionMonitor)(); },
                                                  [converter = differentialSpeedConverter](double speed) { converter->setPitchSpeed(speed); },
                                                  n)});
    namedJointMap.insert({"wristRoll_link", std::make_unique<Joint>(
                                                "wristRoll",
                                                [wristRollPositionMonitor=namedPositionMonitors.at("wristRoll_link")]() -> double { return (*wristRollPositionMonitor)(); },
                                                [converter = differentialSpeedConverter](double speed) { converter->setRollSpeed(speed); },
                                                n)});

    // Initialize the Action Server
    Server server(
        n, "/arm_controller/follow_joint_trajectory",
        [&server](auto goal) { execute(goal, &server); }, false);
    // Start the Action Server
    server.start();
    std::cout << "server started" << std::endl;

    // Create a service to re-enable IK if it gets disabled
    enableServiceServer = n.advertiseService<std_srvs::Trigger>(
        "start_IK",
        [](std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) -> bool {
            IKEnabled = true;
            res.message = "Arm IK Enabled";
            res.success = static_cast<unsigned char>(true);
            return true;
        });

    // Periodically check if overrucrrent faults have occurred
    ros::Timer currentTimer = n.createTimer(
        ros::Duration{OVERCURRENT_TIMEOUT_PERIOD_SEC}, 
        [&motors](const ros::TimerEvent& event) { checkOverCurrentFaults(motors); });

    std::cout << "entering ROS spin..." << std::endl;
    // ROS spin for communication with other nodes
    ros::spin();
    // Return 0 on exit (successful exit)
    return 0;
}
/// @}
