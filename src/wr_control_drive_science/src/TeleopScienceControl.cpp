#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"

constexpr std::uint32_t MESSAGE_CACHE_SIZE = 10;

int main(int argc, char** argv) {
    ros::init(argc, argv, "Science Teleop Control");

    ros::NodeHandle n;

    ros::Publisher screwLiftPow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTablePow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher linearActuatorPow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux1/cmd/left", MESSAGE_CACHE_SIZE);
    ros::Publisher clawPow = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux0/cmd/right", MESSAGE_CACHE_SIZE);
    ros::Publisher turnTableEncoder = n.advertise<std_msgs::Float64>("pid/turnTable", MESSAGE_CACHE_SIZE);

    ros::Subscriber encoderSub = n.subscribe("/hsi/roboclaw/aux0/enc/right", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::UInt32::ConstPtr&)>>(
                [&turnTableEncoder](const std_msgs::UInt32::ConstPtr& msg) {
                    std_msgs::Float64 out;
                    out.data = msg->data;
                    turnTableEncoder.publish(out);
                }
    ));
    ros::Subscriber screwLiftSub = n.subscribe("logic/science/screwLift", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&screwLiftPow](const std_msgs::Float32::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    screwLiftPow.publish(out);
                }
    ));
    ros::Subscriber turnTableSub = n.subscribe("pid/turnTablePow", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float64::ConstPtr&)>>(
                [&turnTablePow](const std_msgs::Float64::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    turnTablePow.publish(out);
                }
            )); 
            
    ros::Subscriber linearActuatorSub = n.subscribe("logic/science/linearActuator", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&linearActuatorPow](const std_msgs::Float32::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    linearActuatorPow.publish(out);
                }
    ));
    ros::Subscriber clawSub = n.subscribe("logic/science/claw", MESSAGE_CACHE_SIZE,
            static_cast<boost::function<void(const std_msgs::Float32::ConstPtr&)>>(
                [&clawPow](const std_msgs::Float32::ConstPtr& msg) {
                    std_msgs::Int16 out;
                    out.data = msg->data * INT16_MAX;
                    clawPow.publish(out);
                }
    ));

    ros::spin();
}
    