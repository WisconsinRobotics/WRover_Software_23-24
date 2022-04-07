#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Science Teleop Logic");

    ros::NodeHandle n;
    
    auto screwLiftMsg = n.advertise<std_msgs::Float32>("/logic/science/screwLift", MESSAGE_CACHE_SIZE);
    auto turnTableMsg = n.advertise<std_msgs::Float32>("/logic/science/turnTableL", MESSAGE_CACHE_SIZE);
    auto linearActuatorMsg = n.advertise<std_msgs::Float32>("/logic/science/linearActuactor", MESSAGE_CACHE_SIZE);
    auto clawMsg = n.advertise<std_msgs::Float32>("/logic/science/claw", MESSAGE_CACHE_SIZE);

    auto screwLiftControl = n.subscribe("/hci/science/gamepad/axis/pov_y", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftMsg](std_msgs::Float32::ConstPtr& msg) {
                    screwLiftMsg.publish(msg);
                }
    ));
    auto turnTableControlL = n.subscribe("/hci/science/gamepad/axis/shoulder_l", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&turnTableMsg](std_msgs::Bool::ConstPtr& msg) {
                    turnTableMsg.publish(msg);
                }
    ));
    auto turnTableControlR = n.subscribe("/hci/science/gamepad/axis/shoulder_r", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&turnTableMsg](std_msgs::Bool::ConstPtr& msg) {
                    turnTableMsg.publish(msg);
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