// Header guard
#ifndef CLAW_CONTROLLER_HPP
#define CLAW_CONTROLLER_HPP

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "std_msgs/Bool.h"

class ClawController{
public:
    explicit ClawController(ros::NodeHandle&);

    /**
     * @brief This method should open the claw, and should be called in the callbacks
     * of the A button

     */
    void openClaw(const std_msgs::Bool::ConstPtr& msg);

    /**
     * @brief This method should close the claw, and should be called in the callbacks
     * of the B button
     * 
     */
    void closeClaw(const std_msgs::Bool::ConstPtr& msg);

    void checkMessage();

private:
    ros::Publisher pub;
    ros::Subscriber openASub;

    ros::Subscriber closeBSub;
    bool aPressed;
    bool bPressed;


};

#endif