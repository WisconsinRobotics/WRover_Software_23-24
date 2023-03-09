#include "ClawController.hpp"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int16.h"

// Again, you may want a constructor here
// For pass-by-value reasons, you may want to take the NodeHandle as an argument
// And construct the Publisher in this constructor, rather that taking the Publisher as 
// an argument.

constexpr uint32_t MESSAGE_QUEUE_LENGTH = 1000;
constexpr int16_t openSpeed = 32767;
constexpr int16_t closeSpeed = -32768;

ClawController::ClawController(ros::NodeHandle& n) : 
    openAPub(n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", 
        MESSAGE_QUEUE_LENGTH)), 
    openASub(n.subscribe("/hci/arm/gamepad/button/a", 
        MESSAGE_QUEUE_LENGTH, &ClawController::openClaw, this)),
    closeBPub(n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", 
        MESSAGE_QUEUE_LENGTH)), 
    closeBSub(n.subscribe("/hci/arm/gamepad/button/b", 
        MESSAGE_QUEUE_LENGTH, &ClawController::closeClaw, this)),
    aPressed(false), bPressed(false) {}

void ClawController::openClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should open the claw
    // ROS_INFO("I heard: [%s]", msg->data);
    this->aPressed = (msg->data != 0U);

}

void ClawController::closeClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should close the claw
    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    this->bPressed = (msg->data != 0U);
}

void ClawController::checkMessage()
{
    while (ros::ok())
    {
        if (aPressed && bPressed)
        {
            std_msgs::Int16 msgNA;
            msgNA.data = 0;
            openAPub.publish(msgNA);
            closeBPub.publish(msgNA);
            // ROS_INFO("%i", msgNA.data);
        }
        else if (aPressed)
        {
            std_msgs::Int16 msgA;
            msgA.data = openSpeed;
            openAPub.publish(msgA);
            // ROS_INFO("%i", msgA.data);
        }
        else
        {
            std_msgs::Int16 msgB;
            msgB.data = closeSpeed;
            closeBPub.publish(msgB);
            // ROS_INFO("%i", msgB.data);
        }
    }
}