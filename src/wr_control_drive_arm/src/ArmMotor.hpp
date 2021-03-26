#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "math.h"
#include <string>

enum MotorState{
    STOP, MOVING, RUN_TO_TARGET
};
class ArmMotor{
    private:
        MotorState currState;
        std::string motorName;
        unsigned int controllerID;
        unsigned int motorID;
        unsigned int encoderVal;
        ros::Subscriber encRead;
        ros::Publisher speedPub;
        static int radToEnc(float rad);
        void storeEncoderVals(const std_msgs::UInt32::ConstPtr& msg);
    public:
        ArmMotor();
        ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle* n);
        // ~ArmMotor();
        int getEncoderCounts();
        // void resetEncoder();
        void runToTarget(int targetCounts, float power);
        void runToTarget(int targetCounts, float power, bool block);
        void runToTarget(double rads, float power);
        MotorState getMotorState();
        void setPower(float power);
        double getRads();
        std::string getMotorName();
        bool hasReachedTarget(int targetCounts);
};