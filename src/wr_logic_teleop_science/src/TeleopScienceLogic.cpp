#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include <array>

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Science Teleop Logic");

    ros::NodeHandle n;
    ros::NodeHandle np{"~"};
    std::vector<int> turnTablePositions;
    np.getParam("turnTablePositions", turnTablePositions);
    bool canListenL = true;
    bool canListenR = true;
    int setpoint = 0;
     
    auto screwLiftMsg = n.advertise<std_msgs::Float32>("/logic/science/screwLift", MESSAGE_CACHE_SIZE);
    auto turnTableMsg = n.advertise<std_msgs::UInt32>("/logic/science/turnTable", MESSAGE_CACHE_SIZE);
    auto linearActuatorMsg = n.advertise<std_msgs::Float32>("/logic/science/linearActuactor", MESSAGE_CACHE_SIZE);
    auto clawMsg = n.advertise<std_msgs::Float32>("/logic/science/claw", MESSAGE_CACHE_SIZE);

    auto screwLiftControl = n.subscribe("/hci/science/gamepad/axis/pov_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftMsg](std_msgs::Float32::ConstPtr& msg) {
                    screwLiftMsg.publish(msg);
                }
    ));
    auto turnTableControlL = n.subscribe("/hci/science/gamepad/axis/shoulder_l", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Bool::ConstPtr&)>>(
                [&turnTableMsg, &turnTablePositions, &canListenL, &setpoint](std_msgs::Bool::ConstPtr& msg) {
                    if(msg->data) {
                        if(canListenL) {
                            setpoint = (setpoint + turnTablePositions.size() - 1) % turnTablePositions.size();
                            std_msgs::UInt32 x;
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
                [&turnTableMsg, &turnTablePositions, &canListenR, &setpoint](std_msgs::Bool::ConstPtr& msg) {
                    if(msg->data) {
                        if(canListenR) {
                            setpoint = (setpoint + turnTablePositions.size() + 1) % turnTablePositions.size();
                            std_msgs::UInt32 x;
                            x.data = turnTablePositions.at(setpoint);
                            turnTableMsg.publish(x);
                            canListenR = false;
                        }
                    } else {
                    canListenR = true;
                    }
                }
    )); 
    auto linearActuactorControl = n.subscribe("hci/science/gamepad/axis/stick_left_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&linearActuatorMsg](std_msgs::Float32::ConstPtr& msg) {
                    linearActuatorMsg.publish(msg);
                }
    ));
    auto clawControl = n.subscribe("hci/science/gamepad/axis/stick_right_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&clawMsg](std_msgs::Float32::ConstPtr& msg) {
                    clawMsg.publish(msg);
                }
    ));

    ros::spin();
}