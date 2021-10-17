/**
 * @file DifferentailJoint.cpp
 * @author Nicholas Underwood
 * @brief The implementation of the DifferentailJoint class
 * @date 2021-10-17
 */

#include "DifferentialJoint.hpp"


void DifferentialJoint::redirectRollPowerOutput(const Std_Float64 msg){
    // Set the speed to be the contained data
    this->rollPower = msg->data;
    this->publishMotorPowers();
}

void DifferentialJoint::redirectPitchPowerOutput(const Std_Float64 msg){
    // Set the speed to be the contained data
    this->pitchPower = msg->data;
    this->publishMotorPowers();
}

vector< DifferentialJoint::publishMotorPowers(){
    // Set the speed to be the contained data
    
}



/// controllerID is constrained between [0,3]
/// motorID is constrained between [0,1]
DifferentialJoint::DifferentialJoint(ArmMotor* motor1, ArmMotor* motor1, ros::NodeHandle* n){
    
    // Set the current name, controller, and motor ID
    this->motor1 = *motor1;
    this->motor2 = *motor2;

    this->pitchPower = 0;
    this->rollPower = 0;

    // Create the topic string prefix for WRoboclaw controllers
    // std::string tpString = ((std::string)"/hsi/roboclaw/aux") + std::to_string(controllerID);
    std::string rollControlString1 = "/control/arm/roll";
    std::string pitchControlString2 = "/control/arm/pitch";
    std::string targetString1 = "/control/arm/" + std::to_string(this->motor1.getControllerID()) + std::to_string(this->motor1.getMotorID());
    std::string targetString2 = "/control/arm/" + std::to_string(this->motor2.getControllerID()) + std::to_string(this->motor2.getMotorID());

    // Create the appropriate mocked encoder-reading and speed-publishing subscribers and advertisers, respectfully
    // this->encRead = n->subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), 1000, &ArmMotor::storeEncoderVals, this);
    // this->speedPub = n->advertise<std_msgs::Int16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), 1000);
    this->targetPub1 = n->advertise<std_msgs::Float64>(controlString1 + "/setpoint", 1000);
    this->targetPub2 = n->advertise<std_msgs::Float64>(controlString2 + "/setpoint", 1000);
    this->feedbackPub1 = n->advertise<std_msgs::Float64>(controlString1 + "/feedback", 1000);
    this->feedbackPub2 = n->advertise<std_msgs::Float64>(controlString2 + "/feedback", 1000);
    this->outputRead = n->subscribe(controlString + "/output", 1000, &ArmMotor::redirectLeftPowerOutput, this);
    this->outputRead = n->subscribe(controlString + "/output", 1000, &ArmMotor::redirectRightPowerOutput, this);
}

