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
     * 
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
    // You may want to keep the speed ros::Publisher as a member variable to publish
    // from the member methods
    ros::Publisher speedPublisher;
    bool aPressed;
    bool bPressed;
    // If you store a publisher, you may want to construct that publisher in the
    // constructor of the class

};

#endif