/**
 * @file ArmMotor.hpp
 * @author Ben Nowotny
 * @brief ablskjlfkejfs
 * @date 2021-04-05
 */
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Float64.h"
#include "math.h"
#include <string>

/**
 * @brief An enumeration of states for a motor to be in.
 */
enum class MotorState{
    /// A Motor is stopped (not moving, 0 power command)
    STOP,
    /// A Motor is moving (non-0 power command)
    MOVING,
    /// A Motor is running to a given target
    RUN_TO_TARGET
};
/**
 * @brief A way to control arm motors with WRoboclaw
 */
class ArmMotor{
    private:
        /// Tolerance ratio w.r.t the counts per rotation for encoder run-to-position motion
        static constexpr double TOLERANCE_RATIO = 0.1/360;
        /// Size of ROS Topic message caches
        static constexpr uint32_t MESSAGE_CACHE_SIZE = 10;
        /// The number of encoder counts per rotation
        int64_t const COUNTS_PER_ROTATION;
        /// The upper and lower bounds of encoder rotation for absolute encoders (index 0 is lower, index 1 is upper)
        std::array<int64_t const, 2> const ENCODER_BOUNDS;
        /// zero position of motor
        int64_t const ENCODER_OFFSET;
        /// The current state of the motor
        MotorState currState;
        /// The joint name of the current motor
        std::string motorName;
        /// The ID of the WRoboclaw controller
        unsigned int controllerID;
        /// The ID of the motor within the WRoboclaw controller
        unsigned int motorID;
        /// The current encoder value
        uint32_t encoderVal;
        /// The ROS Subscriber that reads the encoders
        ros::Subscriber encRead;
        /// The ROS Publisher that sets the encoder targets
        ros::Publisher targetPub;
        /// The ROS Publisher that sets encoder feedback data
        ros::Publisher feedbackPub;
        /// The ROS Subscriber that reads controlled output data
        ros::Subscriber outputRead;
        /// The ROS Publisher that publishes motor speed commands
        ros::Publisher speedPub;
        /// The most recent power message sent
        std_msgs::Int16 powerMsg;

        /**
         * @brief A static conversion from radians to encoder counts
         * 
         * @param rad The input number of radians
         * @return uint32_t The corresponding encoder count bounded by ENCODER_BOUNDS
         */
        uint32_t radToEnc(double rad) const;

        /**
         * @brief A static conversion from encoder counts to radians
         * 
         * @param enc The input number of encoder counts
         * @return double The corresponding radian measure
         */
        double encToRad(uint32_t enc) const;

        /**
         * @brief Subscriber callback for encRead, captures the encoder value of the current motor
         * 
         * @param msg The encoder value message as captured by encRead 
         */
        void storeEncoderVals(const std_msgs::UInt32::ConstPtr& msg);

        /**
         * @brief Subscriber callback for outputRead, captures the PID output and sets the speed directly
         * 
         * @param msg The PID output as captured by outputRead
         */
        void redirectPowerOutput(const std_msgs::Float64::ConstPtr& msg);

        /**
         * @brief Performs Euclidean correct modulus between two inputs of the same type
         * 
         * @param i The dividend of the modulus
         * @param j The divisor of the modulus
         * @return double The Euclidean-correct remainder bounded on [0, j)
         */
        static double corrMod(double i, double j);

    public:
        /**
         * @brief Constructs a new ArmMotor
         * 
         * @param motorName The joint name of the motor
         * @param controllerID The WRoboclaw controller ID for this motor
         * @param motorID The motor ID within its WRoboclaw controller
         * @param n A NodeHandle reference to the constructing Node
         */
        ArmMotor(const std::string &motorName, unsigned int controllerID, unsigned int motorID, int64_t countsPerRotation, int64_t offset, ros::NodeHandle &n);

        /**
         * @brief Gets the encoder value of the motor
         * 
         * @return uint32_t The current encoder value of the motor
         */
        uint32_t getEncoderCounts() const;

        /**
         * @brief Sends the motor to run to a target encoder value at a given power without blocking
         * 
         * @param targetCounts The target encoder value for the motor
         * @param power The power to move the motor at (Bounded between [-1, 1])
         */
        void runToTarget(uint32_t targetCounts, float power);

        /**
         * @brief Sends the motor to run to a target encoder value at a given power
         * 
         * @param targetCounts The target encoder value for the motor
         * @param power The power to move the motor at (Bounded between [-1, 1])
         * @param block Specifies whether or not this action should block until it is complete
         */
        void runToTarget(uint32_t targetCounts, float power, bool block);

        /**
         * @brief Sends the motor to run to a specified position at a given power
         * 
         * @param rads The position to send the motor to (specified in radians)
         * @param power The power to move the motor at (Bounded between [-1, 1])
         */
        void runToTarget(double rads, float power);

        /**
         * @brief Get the current state of the ArmMotor
         * 
         * @return MotorState The current state of the ArmMotor
         */
        MotorState getMotorState() const;

        /**
         * @brief Set the motor power
         * 
         * @param power The power to set the motor at (Bounded between [-1, 1])
         */
        void setPower(float power);

        /**
         * @brief Get the radian measure of the current motor
         * 
         * @return double The radian measure of the current motor's position
         */
        double getRads() const;

        /**
         * @brief Get the name of the ArmMotor
         * 
         * @return std::string The name of the ArmMotor
         */
        std::string getMotorName() const;

        /**
         * @brief Get the name of the ArmMotor
         * 
         * @return std::string The controller ID of the ArmMotor
         */
        unsigned int getControllerID() const;

        /**
         * @brief Get the name of the ArmMotor
         * 
         * @return std::string The motor ID of the ArmMotor
         */
        unsigned int getMotorID() const;

        /**
         * @brief Checks if the motor is currently within a pre-specified tolerance of a target
         * 
         * @param targetCounts The target to test against
         * @return true The motor was within the target tolerance
         * @return false The motor was outside of the target tolerance
         */
        bool hasReachedTarget(uint32_t targetCounts) const;

        /**
         * @brief Checks if the motor is currently within a given tolerance of a target
         * 
         * @param targetCounts The target to test against
         * @param tolerance The tolerance to give when testing the target
         * @return true The motor was within the target tolerance
         * @return false The motor was outside the target tolerance
         */
        bool hasReachedTarget(uint32_t targetCounts, uint32_t tolerance) const;

        /**
         * @brief Get the most recently set motor power
         * 
         * @return float Last motor power setting
         */
        float getPower() const;
};
