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
    pub{n.advertise<std_msgs::Int16>("/hsi/roboclaw/aux3/cmd/left", 
        MESSAGE_QUEUE_LENGTH)}, 
    openASub{n.subscribe("/hci/arm/gamepad/button/a", 
        MESSAGE_QUEUE_LENGTH, &ClawController::openClaw, this)},
    closeBSub{n.subscribe("/hci/arm/gamepad/button/b", 
        MESSAGE_QUEUE_LENGTH, &ClawController::closeClaw, this)},
    aPressed{false}, 
    bPressed{false} {}

void ClawController::openClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should open the claw

    this->aPressed = (msg->data != 0U);
    checkMessage();
}

void ClawController::closeClaw(const std_msgs::Bool::ConstPtr& msg)
{
    // This should close the claw

    this->bPressed = (msg->data != 0U);
    checkMessage();
}

void ClawController::checkMessage()
{

    if (aPressed && bPressed)
    {
        std_msgs::Int16 msgNA;
        msgNA.data = 0;
        pub.publish(msgNA);
    }
    else if (aPressed)
    {
        std_msgs::Int16 msgA;
        msgA.data = openSpeed;
        pub.publish(msgA);
    }
    else
    {
        std_msgs::Int16 msgB;
        msgB.data = closeSpeed;
        pub.publish(msgB);
    }

}