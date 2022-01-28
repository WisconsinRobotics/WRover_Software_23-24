#ifndef SIMPLE_JOINT_GUARD
#define SIMPLE_JOINT_GUARD
/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "SimpleJoint.hpp"

SimpleJoint::SimpleJoint(ArmMotor* motor, ros::NodeHandle *n) : AbstractJoint(n, 1) {
    this->motors.push_back(motor);
}

void SimpleJoint::getJointPositions(vector<double> &motorPositions, vector<double> &target){
    // vector<double> positions = *(new vector<double>()); // makes the red lines go away
    
    // target->reserve(1);
    target.push_back(motorPositions[0]);

    // return positions;
}

void SimpleJoint::getMotorPositions(vector<double> &jointPositions, vector<double> &target){
  
    target.reserve(1);
    std::cout << target.size() << std::endl;
    double position = jointPositions[0];
    target.push_back(position);
    // return positions;
}

void SimpleJoint::getMotorVelocities(vector<double> &jointPositions, vector<double> &target){
    
    // target->reserve(1);
    target.push_back(jointPositions[0]);

    // return setpoints;
}

void SimpleJoint::handoffOutput(const std_msgs::Float64 msg){
    this->motorOutputPublisher.publish(msg);
}

void SimpleJoint::handoffFeedback(const std_msgs::Float64 msg){
    this->jointFeedbackPublisher.publish(msg);
}
#endif