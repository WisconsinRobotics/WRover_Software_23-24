#include "ArmMotor.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "math.h"

#define Std_UInt32 std_msgs::UInt32::ConstPtr&

class ArmMotor{
    private:
        MotorState currState;
        std::string motorName;
        unsigned int controllerID;
        unsigned int motorID;
        ros::Subscriber encRead;
        ros::Publisher speedPub;
        int encoderVal;

        static const int COUNTS_PER_ROTATION = UINT_FAST32_MAX;

        static float radToEnc(float rads){
            return COUNTS_PER_ROTATION * rads / (2 * M_PI);
        }

        void storeEncoderVals(const Std_UInt32 msg){
            this->encoderVal = msg->data;
        }

    public:
        ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle& n){
            if(controllerID > 3) throw ((std::string)"Controller ID ") + std::to_string(controllerID) + "is not valid on [0,3]";
            if(motorID > 1) throw ((std::string)"Motor ID ") + std::to_string(motorID) + "is not valid on [0,1]";
            
            this->motorName = motorName;
            this->controllerID = controllerID;
            this->motorID = motorID;
            this->currState = MotorState::STOP;

            std::string tpString = ((std::string)"/hsi/wroboclaw/aux") + std::to_string(controllerID);

            encRead = n.subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), 1000, storeEncoderVals, this);
            speedPub = n.advertise<std_msgs::UInt16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), 1000);
        }

        // ~ArmMotor();
        
        int getEncoderCounts(){
            return encoderVal;
        }

        // void resetEncoder();

        void runToTarget(int targetCounts, float power){
            this->runToTarget(targetCounts, power, false);
        }

        bool hasReachedTarget(int targetCounts){
            return abs(targetCounts - this->getEncoderCounts()) < 10;
        }

        MotorState getMotorState(){
            return this->currState;
        }

        void setPower(float power){
            if(abs(power) > 1) throw ((std::string)"Power ") + std::to_string(power) + " is not on the interval [-1, 1]";

            std_msgs::UInt16 msg;
            msg.data = power * INT16_MAX;
            this->speedPub.publish(msg);
            this->currState = power == 0.f ? MotorState::STOP : MotorState::MOVING;
        }

        void runToTarget(int targetCounts, float power, bool block){
            if(this->getMotorState() != MotorState::RUN_TO_TARGET){
                power = abs(power) * abs(targetCounts - this->getEncoderCounts())/(targetCounts - this->getEncoderCounts());
                this->setPower(power);
                this->currState = MotorState::RUN_TO_TARGET;
            }
            if(this->hasReachedTarget(targetCounts)){
                this->setPower(0.f);
                this->currState = MotorState::STOP;
            }
            if(block){
                while(!this->hasReachedTarget(targetCounts));
                this->setPower(0.f);
                this->currState = MotorState::STOP;
            }
        }

        void runToTarget(double rads, float power){
            runToTarget(this->radToEnc(rads), power, false);
        }
};