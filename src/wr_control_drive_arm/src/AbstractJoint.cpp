/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */
#include "AbstractJoint.hpp"

AbstractJoint::AbstractJoint(ros::NodeHandle &n, int numMotors){
    this->motors.resize(numMotors);
}

unsigned int AbstractJoint::getDegreesOfFreedom() const{
    return this->motors.size();
}

const std::unique_ptr<ArmMotor>& AbstractJoint::getMotor(int motorIndex) const{
    return this->motors.at(motorIndex).motor;
}

void AbstractJoint::configSetpoint(int degreeIndex, double position, double velocity){

    this->motors.at(degreeIndex).position = position;
    this->motors.at(degreeIndex).velocity = velocity;
}


bool AbstractJoint::exectute(){
    for(auto &motorHandle : motors){
        if (!motorHandle.motor->runToTarget(motorHandle.position, 0)) 
            return false;
    }
    return true;
}