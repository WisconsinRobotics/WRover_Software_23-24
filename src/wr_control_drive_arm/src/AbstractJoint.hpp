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
        std::string name;

    public:

        AbstractJoint(ros::NodeHandle* n, std::string name=""){
            this->n = n;
            this->name = name;

            std::cout << this->numMotors << std::endl;
            for(int i = 0; i < 1; i++){

                jointPositions.push_back(0);
                jointVelocites.push_back(0);
            }
        };
        ~AbstractJoint(){};

        std::string getName() { return this->name; }


        // never used, need to be defined for compiler v-table
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
            std::cout << "config setpoint: " << degreeIndex << " " << position << " " << velocity << std::endl;
            std::cout << "capacity: " << this->jointPositions.size() << std::endl;

            this->jointPositions[degreeIndex] = position;
            this->jointVelocites[degreeIndex] = velocity;
        }

        void exectute(){
            vector<double> motorPositions = this->getMotorPositions(jointPositions);
            for(int i = 0; i < this->numMotors; i++){
                std::cout << "run motor to target: " << motorPositions[i] << std::endl;
                this->motors[i]->runToTarget(motorPositions[i], 0);
            }
        }

        virtual void configVelocityHandshake(std::string joint_topic, std::string motor_topic){};

};