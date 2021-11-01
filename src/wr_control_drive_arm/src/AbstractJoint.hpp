/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "ArmMotor.hpp"
using std::vector;

struct PositionVelocityPair {
    double position;
    double velocity;
};

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

        AbstractJoint(ros::NodeHandle* n){
            this->n = n;
        };
        ~AbstractJoint(){};


        virtual vector<double> getMotorPositions(vector<double> jointPositions){
            return *(new vector<double>());
        }
        virtual vector<double> getMotorVelocities(vector<double> joinVelocities){
            return *(new vector<double>());
        } 
        virtual vector<double> getJointPositions(vector<double> motorPositions){
            return *(new vector<double>());
        }

        unsigned int getDegreesOfFreedom(){
            return this->numMotors;
        }
        
        ArmMotor* getMotor(int motorIndex){
            return this->motors[motorIndex];
        }

        void configSetpoint(int degreeIndex, double position, double velocity){
            this->jointPositions[degreeIndex] = position;
            this->jointVelocites[degreeIndex] = velocity;
        }

        void exectute(){
            vector<double> motorPositions = this->getMotorPositions(jointPositions);

            for(int i = 0; i < this->numMotors; i++){
                this->motors[i]->runToTarget(motorPositions[i], 0);
            }
        }

        virtual void configVelocityHandshake(std::string joint_topic, std::string motor_topic){};

};