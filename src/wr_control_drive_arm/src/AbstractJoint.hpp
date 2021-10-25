/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-04-05
 */
#include <iostream>
// #include "ros/ros.h"
// #include "std_msgs/UInt32.h"
// #include "std_msgs/Int16.h"
// #include "std_msgs/Float64.h"
#include "ArmMotor.hpp"
#include "math.h"
#include <string>

class AbstractJoint {
    private:
        unsigned int degreesOfFreedom;
        ros::Subscriber velocityReads[];
        ArmMotor

    public:
        unsigned int getDegreesOfFreedom();
};