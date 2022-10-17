/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief Header file of the AbstractJoint class
 * @date 2021-10-25
 */

#ifndef ABSTRACT_JOINT_GUARD
#define ABSTRACT_JOINT_GUARD

#include "ArmMotor.hpp"
using std::vector;

class AbstractJoint {
public:
    struct MotorHandler{
        std::unique_ptr<ArmMotor> motor;
        double position;
        double velocity;
        std::string jointTopicName;
        std::string motorTopicName;
        bool newVelocity;
    };

    AbstractJoint(ros::NodeHandle &n, int numMotors);

    // never used, need to be defined for compiler v-table
    virtual auto getMotorPositions(const vector<double> &jointPositions) -> vector<double> = 0;
    virtual auto getMotorVelocities(const vector<double> &joinVelocities) -> vector<double> = 0;
    virtual auto getJointPositions(const vector<double> &motorPositions) -> vector<double> = 0;

    auto getDegreesOfFreedom() const -> unsigned int;
    
    auto getMotor(int motorIndex) const -> const std::unique_ptr<ArmMotor>&;

    void configSetpoint(int degreeIndex, double position, double velocity);

    void exectute();

    void stopJoint();

protected:
    vector<MotorHandler> motors;
};

#endif