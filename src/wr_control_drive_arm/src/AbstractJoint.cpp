/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */
#include "AbstractJoint.hpp"

AbstractJoint::AbstractJoint(ros::NodeHandle* n){
    this->n = n;

    std::cout << this->numMotors << std::endl;
    for(int i = 0; i < 1; i++){

        jointPositions.push_back(0);
        jointVelocites.push_back(0);
    }
}

int AbstractJoint::getDegreesOfFreedom(){
    return this->numMotors;
}

ArmMotor* AbstractJoint::getMotor(int motorIndex){
    return this->motors[motorIndex];
}

void AbstractJoint::configSetpoint(int degreeIndex, double position, double velocity){
    std::cout << "config setpoint: " << degreeIndex << " " << position << " " << velocity << std::endl;
    std::cout << "capacity: " << this->jointPositions.size() << std::endl;

    this->jointPositions[degreeIndex] = position;
    this->jointVelocites[degreeIndex] = velocity;
}

bool AbstractJoint::exectute(){
    vector<double> motorPositions = this->getMotorPositions(jointPositions);
    for(int i = 0; i < this->numMotors; i++){
        std::cout << "run motor to target: " << motorPositions[i] << std::endl;
        if (!this->motors[i]->runToTarget(motorPositions[i], 0)) return false;
    }
    return true;
}