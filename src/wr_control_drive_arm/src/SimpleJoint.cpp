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

auto SimpleJoint::getMotorPositions(const vector<double> &jointPositions) -> vector<double> {
    return {jointPositions[0]};
}

auto SimpleJoint::getMotorVelocities(const vector<double> &jointVelocities) -> vector<double> {
    return {jointVelocities[0]};
}

auto SimpleJoint::getJointPositions(const vector<double> &jointPositions) -> vector<double> {
    return {jointPositions[0]};
}