#include "ClawController.hpp"
#include "ros/node_handle.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

// Again, you may want a constructor here
// For pass-by-value reasons, you may want to take the NodeHandle as an argument
// And construct the Publisher in this constructor, rather that taking the Publisher as 
// an argument.

constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;

ClawController::ClawController(ros::NodeHandle& n)
{
    ros::Publisher openAPub = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", MESSAGE_QUEUE_LENGTH);
    ros::Publisher closeBPub = n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", MESSAGE_QUEUE_LENGTH);
    ros::Subscriber openASub = n.subscribe("/hci/arm/gamepad/button/a", MESSAGE_QUEUE_LENGTH, &ClawController::openClaw, this);
    ros::Subscriber closeBSub = n.subscribe("/hci/arm/gamepad/button/b", MESSAGE_QUEUE_LENGTH, &ClawController::closeClaw, this);
}

void ClawController::openClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should open the claw
    // ROS_INFO("I heard: [%s]", msg->data);
    this->aPressed = (msg->data != 0U);
    
}

void ClawController::closeClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should close the claw
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void ClawController::checkMessage()
{
    if (this->openClaw() == )
    {
        std_msgs::Int16 msgNA;
        msgNA.data = 0;
        openAPub.publish(msgNA);
        closeBPub.publish(msgNA);
    }
    else if ()
    {
        std_msgs::Int16 msgA;
        msgA.data = 32767;
        openAPub.publish(msgA);
    }
    else
    {
        std_msgs::Int16 msgB;
        msgB.data = -32768;
        closeBPub.publish(msgB);
    }
}