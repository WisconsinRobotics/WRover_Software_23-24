#ifndef ABSTRACT_JOINT_GUARD
#define ABSTRACT_JOINT_GUARD

#include "ArmMotor.hpp"
using std::vector;

class AbstractJoint {


    protected:
        ros::NodeHandle* n;
        unsigned int numMotors;
        vector<ArmMotor*> motors;

        vector<double> jointPositions;   
        vector<double> jointVelocites;

        vector<std::string> jointTopicsNames;  
        vector<std::string> motorTopicsNames;  
        vector<bool> newVelocitiesVector;

    public:

        AbstractJoint(ros::NodeHandle* n);
        ~AbstractJoint(){};

        // never used, need to be defined for compiler v-table
        virtual vector<double> getMotorPositions(vector<double> jointPositions) = 0;
        virtual vector<double> getMotorVelocities(vector<double> joinVelocities) = 0;
        virtual vector<double> getJointPositions(vector<double> motorPositions) = 0;

        int getDegreesOfFreedom();
        
        ArmMotor* getMotor(int motorIndex);

        void configSetpoint(int degreeIndex, double position, double velocity);

        bool exectute();

        // virtual void configVelocityHandshake(std::string, std::string) = 0;
};

#endif