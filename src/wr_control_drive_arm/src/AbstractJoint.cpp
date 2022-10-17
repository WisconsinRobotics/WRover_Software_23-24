/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief Implementation of the AbstractJoint class
 * @date 2021-10-25
 */
 
#include "AbstractJoint.hpp"

AbstractJoint::AbstractJoint(ros::NodeHandle &n, int numMotors){
    this->motors.resize(numMotors);
}

auto AbstractJoint::getDegreesOfFreedom() const -> unsigned int{
    return this->motors.size();
}

auto AbstractJoint::getMotor(int motorIndex) const -> const std::unique_ptr<ArmMotor>&{
    return this->motors.at(motorIndex).motor;
}

void AbstractJoint::configSetpoint(int degreeIndex, double position, double velocity){

    this->motors.at(degreeIndex).position = position;
    this->motors.at(degreeIndex).velocity = velocity;
}


void AbstractJoint::exectute(){
    for(auto &motorHandle : motors){
        motorHandle.motor->runToTarget(motorHandle.position, motorHandle.velocity);
    }
}

void AbstractJoint::stopJoint(){
    for(auto &motorHandle : motors){
        motorHandle.motor->setPower(0.F);
    }
}