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

        AbstractJoint(ros::NodeHandle* n, int numMotors);
        ~AbstractJoint(){};

        // never used, need to be defined for compiler v-table
        virtual void getMotorPositions(vector<double> &jointPositions, vector<double> &target) = 0;
        virtual void getMotorVelocities(vector<double> &joinVelocities, vector<double> &target) = 0;
        virtual void getJointPositions(vector<double> &motorPositions, vector<double> &target) = 0;

        int getDegreesOfFreedom();
        
        ArmMotor* getMotor(int motorIndex);

        void configSetpoint(int degreeIndex, double position, double velocity);

        void exectute();

        // virtual void configVelocityHandshake(std::string, std::string) = 0;
};

#endif