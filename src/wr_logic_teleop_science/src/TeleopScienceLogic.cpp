#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"
#include <array>

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "Science Teleop Logic");

    ros::NodeHandle n;
    ros::NodeHandle np{"~"};
    std::vector<double> turnTablePositions;
    np.getParam("turnTablePositions", turnTablePositions);
    bool canListenL = true;
    bool canListenR = true;
    unsigned long setpoint = 0;
     
    auto screwLiftMsg = n.advertise<std_msgs::Float32>("/logic/science/screwLift", MESSAGE_CACHE_SIZE);
    auto turnTableMsg = n.advertise<std_msgs::Float64>("/logic/science/turnTable", MESSAGE_CACHE_SIZE);
    auto linearActuatorMsg = n.advertise<std_msgs::Float32>("/logic/science/linearActuator", MESSAGE_CACHE_SIZE);
    auto clawMsg = n.advertise<std_msgs::Float32>("/logic/science/claw", MESSAGE_CACHE_SIZE);

    auto screwLiftControl = n.subscribe("/hci/science/gamepad/axis/pov_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftMsg](const std_msgs::Float32::ConstPtr& msg) {
                    screwLiftMsg.publish(msg);
                }
    ));
    auto turnTableControlL = n.subscribe("/hci/science/gamepad/axis/shoulder_l", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Bool::ConstPtr&)>>(
                [&turnTableMsg, &turnTablePositions, &canListenL, &setpoint](const std_msgs::Bool::ConstPtr& msg) {
                    if(static_cast<bool>(msg->data)) {
                        if(canListenL) {
                            setpoint = (setpoint + turnTablePositions.size() - 1) % turnTablePositions.size();
                            std_msgs::Float64 x;
                            x.data = turnTablePositions.at(setpoint);
                            turnTableMsg.publish(x);
                            canListenL = false;
                        }
                    } else {
                    canListenL = true;
                    }
                }
    ));
    auto turnTableControlR = n.subscribe("/hci/science/gamepad/axis/shoulder_r", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Bool::ConstPtr&)>>(
                [&turnTableMsg, &turnTablePositions, &canListenR, &setpoint](const std_msgs::Bool::ConstPtr& msg) {
                    if(static_cast<bool>(msg->data)) {
                        if(canListenR) {
                            setpoint = (setpoint + turnTablePositions.size() + 1) % turnTablePositions.size();
                            std_msgs::Float64 x;
                            x.data = turnTablePositions.at(setpoint);
                            turnTableMsg.publish(x);
                            canListenR = false;
                        }
                    } else {
                    canListenR = true;
                    }
                }
    )); 
    auto linearActuatorControl = n.subscribe("hci/science/gamepad/axis/stick_left_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&linearActuatorMsg](const std_msgs::Float32::ConstPtr& msg) {
                    linearActuatorMsg.publish(msg);
                }
    ));
    auto clawControl = n.subscribe("hci/science/gamepad/axis/stick_right_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&clawMsg](const std_msgs::Float32::ConstPtr& msg) {
                    clawMsg.publish(msg);
                }
    ));

    ros::spin();
}