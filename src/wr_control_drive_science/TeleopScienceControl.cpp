#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Science Teleop Control");

    ros::NodeHandle n;

    ros::Publisher screwLiftPow = n.advertise<int16_t>("/hsi/roboclaw/aux0/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTablePow = n.advertise<int16_t>("/hsi/roboclaw/aux0/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher linearActuatorPow = n.advertise<int16_t>("/hsi/roboclaw/aux1/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher clawPow = n.advertise<int16_t>("/hsi/roboclaw/aux0/cmd/right", MESSAGE_CACHE_SIZE);

    ros::Subscriber screwLiftSub = n.subscribe("logic/science/screwLift", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftPow](std_msgs::Float32::ConstPtr& msg) {
                    int16_t ans = msg->data * .1 * INT16_MAX;
                    screwLiftPow.publish(ans);
                }
    ));
    
    ros::Subscriber linearActuatorSub = n.subscribe("logic/science/linearActuator", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&linearActuatorPow](std_msgs::Float32::ConstPtr& msg) {
                    int16_t ans = msg->data * .1 * INT16_MAX;
                    linearActuatorPow.publish(ans);
                }
    ));
    ros::Subscriber clawSub = n.subscribe("logic/science/screwLift", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&clawPow](std_msgs::Float32::ConstPtr& msg) {
                    int16_t ans = msg->data * .1 * INT16_MAX;
                    clawPow.publish(ans);
                }
    ));

    ros::spin();
}
    