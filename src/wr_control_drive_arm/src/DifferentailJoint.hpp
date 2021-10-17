/**
 * @file DifferentialJoint.hpp
 * @author Nicholas Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-17
 */

#include <iostream>
#include "ros/ros.h"
// #include "std_msgs/UInt32.h"
// #include "std_msgs/Int16.h"
// #include "std_msgs/Float64.h"
#include "math.h"
#include <string>
#include <vector>


#include "ArmMotor.hpp"


/**
 * @brief A way to control arm motors with WRoboclaw
 */
class DifferentialJoint {
    private:
        /// The current roll power
        float rollPower;
        /// The current pitch power
        float pitchPower;
        /// The ROS Subscriber that reads the encoders
        ros::Subscriber encRead;
        /// The ROS Publisher that sets the right encoder targets
        ros::Publisher target1Pub;
        /// The ROS Publisher that sets the left encoder targets
        ros::Publisher target2Pub;
        /// The ROS Publisher that sets encoder feedback data
        ros::Publisher feedbackPub;
        /// The ROS Subscriber that reads controlled output data
        ros::Subscriber outputRead;
        /// The ROS Publisher that publishes motor speed commands
        ros::Publisher speedPub;
        /// A pointer to the most recent power message sent
        std_msgs::Int16 *powerMsg;



    public:
        /**
         * @brief Constructs a new DifferentialJoint
         * 
         * @param motor1 A pointer to the left motor
         * @param motor2 A pointer to the right motor
         * @param n A NodeHandle reference to the constructing Node
         */
        DifferentialJoint(ArmMotor* motor1, ArmMotor* motor2, ros::NodeHandle* n);



        /**
         * @brief gets the left motor of the joint
         * 
         * @return ArmMotor the left motor
         */
        ArmMotor getMotor1();

        /**
         * @brief gets the right motor of the joint
         * 
         * @return ArmMotor the right motor
         */
        ArmMotor getMotor2();

        /**
         * @brief Sends the motor to run to a target encoder value at a given power without blocking
         * 
         * @param targetCounts The target encoder value for the motor
         * @param power The power to move the motor at (Bounded between [-1, 1])
         */
        // void runToTarget(uint32_t targetCounts, float power);

        /**
         * @brief Sends the motor to run to a target encoder value at a given power
         * 
         * @param targetCounts The target encoder value for the motor
         * @param power The power to move the motor at (Bounded between [-1, 1])
         * @param block Specifies whether or not this action should block until it is complete
         */
        // void runToTarget(uint32_t targetCounts, float power, bool block);

        /**
         * @brief Sends the motor to run to a specified position at a given power
         * 
         * @param rads The position to send the motor to (specified in radians)
         * @param power The power to move the motor at (Bounded between [-1, 1])
         */
        // void runToTarget(double rads, float power);

        /**
         * @brief Get the current state of the ArmMotor
         * 
         * @return MotorState The current state of the ArmMotor
         */
        // MotorState getMotorState();

        /**
         * @brief Set the motor power
         * 
         * @param power The power to set the motor at (Bounded between [-1, 1])
         */
        // void setPower(float power);

        /**
         * @brief Get the radian measure of the current motor
         * 
         * @return float The radian measure of the current motor's position
         */
        // float getRads();

        /**
         * @brief Get the name of the ArmMotor
         * 
         * @return std::string The name of the ArmMotor
         */
        // std::string getMotorName();

        /**
         * @brief Checks if the motor is currently within a pre-specified tolerance of a target
         * 
         * @param targetCounts The target to test against
         * @return true The motor was within the target tolerance
         * @return false The motor was outside of the target tolerance
         */
        // bool hasReachedTarget(uint32_t targetCounts);

        /**
         * @brief Checks if the motor is currently within a given tolerance of a target
         * 
         * @param targetCounts The target to test against
         * @param tolerance The tolerance to give when testing the target
         * @return true The motor was within the target tolerance
         * @return false The motor was outside the target tolerance
         */
        // bool hasReachedTarget(uint32_t targetCounts, uint32_t tolerance);
};
