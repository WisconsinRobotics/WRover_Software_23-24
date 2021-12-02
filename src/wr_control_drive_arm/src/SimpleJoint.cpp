#ifndef SIMPLE_JOINT_GUARD
#define SIMPLE_JOINT_GUARD
/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "SimpleJoint.hpp"

SimpleJoint::SimpleJoint(ArmMotor* motor, ros::NodeHandle *n) : AbstractJoint(n) {
    this->numMotors = 1;
    this->motors.push_back(motor);
}

vector<double> SimpleJoint::getJointPositions(vector<double> motorPositions){
    vector<double> positions = *(new vector<double>()); // makes the red lines go away
    
    for(int i = 0; i < motorPositions.size(); i++){
        positions[i] = motorPositions[i];
    }

    return positions;
}

vector<double> SimpleJoint::getMotorPositions(vector<double> jointPositions){
    vector<double> positions;
    for(int i = 0; i < jointPositions.size(); i++){
        positions.push_back(jointPositions[i]);
    }

    return positions;
}

vector<double> SimpleJoint::getMotorVelocities(vector<double> jointPositions){
    vector<double> setpoints;
    
    for(int i = 0; i < jointPositions.size(); i++){
        setpoints[i] = jointVelocites[i];
    }

    return setpoints;
}

void SimpleJoint::handoffOutput(const std_msgs::Float64 msg){
    this->motorOutputPublisher.publish(msg);
}

void SimpleJoint::handoffFeedback(const std_msgs::Float64 msg){
    this->jointFeedbackPublisher.publish(msg);
}
#endif