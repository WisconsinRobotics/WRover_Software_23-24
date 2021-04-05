#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16.h"
#include "math.h"
#include <string>

enum MotorState{
    STOP, MOVING, RUN_TO_TARGET
};
class ArmMotor{
    private:
        static unsigned int const COUNTS_PER_ROTATION;
        static unsigned int const ENCODER_BOUNDS[2];
        MotorState currState;
        std::string motorName;
        unsigned int controllerID;
        unsigned int motorID;
        unsigned int encoderVal;
        ros::Subscriber encRead;
        ros::Publisher speedPub;
        std_msgs::Int16 *powerMsg;
        static unsigned int radToEnc(double rad);
        void storeEncoderVals(const std_msgs::UInt32::ConstPtr& msg);
        template<class T> static T corrMod(T i, T j);
    public:
        ArmMotor();
        ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle* n);
        // ~ArmMotor();
        unsigned int getEncoderCounts();
        // void resetEncoder();
        void runToTarget(unsigned int targetCounts, float power);
        void runToTarget(unsigned int targetCounts, float power, bool block);
        void runToTarget(double rads, float power);
        MotorState getMotorState();
        void setPower(float power);
        float getRads();
        std::string getMotorName();
        bool hasReachedTarget(unsigned int targetCounts);
        bool hasReachedTarget(unsigned int targetCounts, unsigned int tolerance);
        float getPower();
};