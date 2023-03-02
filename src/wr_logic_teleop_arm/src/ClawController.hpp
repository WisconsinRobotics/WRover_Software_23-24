// Header guard
#ifndef CLAW_CONTROLLER_HPP
#define CLAW_CONTROLLER_HPP

class ClawController{
public:
    /**
     * @brief This method should open the claw, and should be called in the callbacks
     * of the A button
     * 
     */
    void openClaw();

    /**
     * @brief This method should close the claw, and should be called in the callbacks
     * of the B button
     * 
     */
    void closeClaw();

private:
    // You may want to keep the speed ros::Publisher as a member variable to publish
    // from the member methods

    // If you store a publisher, you may want to construct that publisher in the
    // constructor of the class

};

#endif