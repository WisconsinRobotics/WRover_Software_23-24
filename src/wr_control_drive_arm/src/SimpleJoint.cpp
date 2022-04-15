/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "SimpleJoint.hpp"

SimpleJoint::SimpleJoint(std::unique_ptr<ArmMotor> motor, ros::NodeHandle &n) : AbstractJoint(n, 1) {
    this->motors.at(0) = MotorHandler{std::move(motor), 0, 0, "", "", false};
}

void SimpleJoint::getJointPositions(const vector<double> &motorPositions, vector<double> &target){
    // vector<double> positions = *(new vector<double>()); // makes the red lines go away
    
    // target->reserve(1);
    target.push_back(motorPositions[0]);

    // return positions;
}

void SimpleJoint::getMotorPositions(const vector<double> &jointPositions, vector<double> &target){
  
    target.reserve(1);
    double position = jointPositions[0];
    target.push_back(position);
    // return positions;
}

void SimpleJoint::getMotorVelocities(const vector<double> &jointPositions, vector<double> &target){
    
    // target->reserve(1);
    target.push_back(jointPositions[0]);

    // return setpoints;
}