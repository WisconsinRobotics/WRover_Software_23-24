#ifndef ABSTRACT_JOINT_GUARD
#define ABSTRACT_JOINT_GUARD

#include "ArmMotor.hpp"
using std::vector;

class AbstractJoint {
public:
    struct MotorHandler{
        const std::unique_ptr<ArmMotor> motor;
        double position;
        double velocity;
        std::string jointTopicName;
        std::string motorTopicName;
        bool newVelocities;
    };

    AbstractJoint(ros::NodeHandle &n, int numMotors);

    // never used, need to be defined for compiler v-table
    virtual void getMotorPositions(vector<double> &jointPositions, vector<double> &target) = 0;
    virtual void getMotorVelocities(vector<double> &joinVelocities, vector<double> &target) = 0;
    virtual void getJointPositions(vector<double> &motorPositions, vector<double> &target) = 0;

    unsigned int getDegreesOfFreedom() const;
    
    const std::unique_ptr<ArmMotor>& getMotor(int motorIndex) const;

    void configSetpoint(int degreeIndex, double position, double velocity);

    bool exectute();

protected:
    vector<MotorHandler> motors;
};

#endif