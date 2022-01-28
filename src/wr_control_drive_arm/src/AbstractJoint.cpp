/**
 * @file AbstractJoint.cpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */
#include "AbstractJoint.hpp"

AbstractJoint::AbstractJoint(ros::NodeHandle* n, int numMotors){
    this->n = n;
    this->numMotors = numMotors;

    jointPositions.reserve(numMotors);
    jointVelocites.reserve(numMotors);

    std::cout << "init with motors: " << this->numMotors << std::endl;
}

int AbstractJoint::getDegreesOfFreedom(){
    return this->numMotors;
}

ArmMotor* AbstractJoint::getMotor(int motorIndex){
    return this->motors[motorIndex];
}

void AbstractJoint::configSetpoint(int degreeIndex, double position, double velocity){
    std::cout << "config setpoint: " << degreeIndex << " " << position << " " << velocity << std::endl;

    this->jointPositions[degreeIndex] = position;
    this->jointVelocites[degreeIndex] = velocity;
}

void AbstractJoint::exectute(){
    std::cout << "exectue" << std::endl;
    vector<double> motorPositions = vector<double>();
    this->getMotorPositions(jointPositions, motorPositions);
    std::cout << motorPositions.size() << std::endl;

    for(int i = 0; i < this->numMotors; i++){
        std::cout << "run motor to target: " << motorPositions[i] << std::endl;
        this->motors[i]->runToTarget(motorPositions[i], 0);
    }
}