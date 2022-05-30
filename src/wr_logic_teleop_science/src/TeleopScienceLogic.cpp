#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"
#include <array>

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "Science Teleop Logic");

    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle{"~"};
    std::vector<double> turnTablePositions;
    privateNodeHandle.getParam("turnTablePositions", turnTablePositions);
    bool canListenL = true;
    bool canListenR = true;
    unsigned long setpoint = 0;
     
    auto screwLiftMsg = nodeHandle.advertise<std_msgs::Float32>("/logic/science/screwLift", MESSAGE_CACHE_SIZE);
    auto turnTableMsg = nodeHandle.advertise<std_msgs::Float64>("/logic/science/turnTable", MESSAGE_CACHE_SIZE);
    auto linearActuatorMsg = nodeHandle.advertise<std_msgs::Float32>("/logic/science/linearActuator", MESSAGE_CACHE_SIZE);
    auto clawMsg = nodeHandle.advertise<std_msgs::Float32>("/logic/science/claw", MESSAGE_CACHE_SIZE);

    auto screwLiftControl = nodeHandle.subscribe("/hci/science/gamepad/axis/pov_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftMsg](const std_msgs::Float32::ConstPtr& msg) {
                    screwLiftMsg.publish(msg);
                }
    ));
    auto turnTableControlL = nodeHandle.subscribe("/hci/science/gamepad/axis/shoulder_l", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Bool::ConstPtr&)>>(
                [&turnTableMsg, &turnTablePositions, &canListenL, &setpoint](const std_msgs::Bool::ConstPtr& msg) {
                    if(static_cast<bool>(msg->data) && canListenL) {
                        setpoint = (setpoint + turnTablePositions.size() - 1) % turnTablePositions.size();
                        std_msgs::Float64 outMsg{};
                        outMsg.data = turnTablePositions.at(setpoint);
                        turnTableMsg.publish(outMsg);
                        canListenL = false;
                    } else {
                    canListenL = true;
                    }
                }
    ));
    auto turnTableControlR = nodeHandle.subscribe("/hci/science/gamepad/axis/shoulder_r", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Bool::ConstPtr&)>>(
                [&turnTableMsg, &turnTablePositions, &canListenR, &setpoint](const std_msgs::Bool::ConstPtr& msg) {
                    if(static_cast<bool>(msg->data) && canListenR) {
                        setpoint = (setpoint + turnTablePositions.size() + 1) % turnTablePositions.size();
                        std_msgs::Float64 outMsg{};
                        outMsg.data = turnTablePositions.at(setpoint);
                        turnTableMsg.publish(outMsg);
                        canListenR = false;
                    } else {
                    canListenR = true;
                    }
                }
    )); 
    auto linearActuatorControl = nodeHandle.subscribe("hci/science/gamepad/axis/stick_left_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&linearActuatorMsg](const std_msgs::Float32::ConstPtr& msg) {
                    linearActuatorMsg.publish(msg);
                }
    ));
    auto clawControl = nodeHandle.subscribe("hci/science/gamepad/axis/stick_right_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&clawMsg](const std_msgs::Float32::ConstPtr& msg) {
                    clawMsg.publish(msg);
                }
    ));

    ros::spin();
}