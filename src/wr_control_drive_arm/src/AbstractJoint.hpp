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
    virtual void getMotorPositions(vector<double> &jointPositions, vector<double> &target) = 0;
    virtual void getMotorVelocities(vector<double> &joinVelocities, vector<double> &target) = 0;
    virtual void getJointPositions(vector<double> &motorPositions, vector<double> &target) = 0;

    auto getDegreesOfFreedom() const -> unsigned int;
    
    auto getMotor(int motorIndex) const -> const std::unique_ptr<ArmMotor>&;

    void configSetpoint(int degreeIndex, double position, double velocity);

    auto exectute() -> bool;

protected:
    vector<MotorHandler> motors;
};

#endif