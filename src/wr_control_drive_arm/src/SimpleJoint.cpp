/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "AbstractJoint.hpp"
using std::vector;

class SimpleJoint : public AbstractJoint {
    public:
        ~SimpleJoint();
        SimpleJoint(ArmMotor* motor, ros::NodeHandle* n);

        int a(){return 2;}
        int b(){return 3;}

        vector<double> getMotorPositions(vector<double> jointPositions);
        vector<double> getMotorVelocities(vector<double> jointVelocities);
        vector<double> getJointPositions(vector<double> motorPositions);

        void configVelocityHandshake(std::string jointTopicName, std::string motorTopicName);
        void handoffOutput(std_msgs::Float64);
        void handoffFeedback(std_msgs::Float64);

    private:
        ros::Subscriber jointOutputSubscriber;
        ros::Publisher motorOutputPublisher;

        ros::Subscriber motorFeedbackSubscriber;
        ros::Publisher jointFeedbackPublisher;
        
};

SimpleJoint::SimpleJoint(ArmMotor* motor, ros::NodeHandle* n) : AbstractJoint(n) {
    this->numMotors = 1;
    this->motors[0] = motor;
}

vector<double> SimpleJoint::getJointPositions(vector<double> motorPositions){
    vector<double> positions = *(new vector<double>()); // makes the red lines go away
    
    for(int i = 0; i < motorPositions.size(); i++){
        positions[i] = motorPositions[i];
    }

    return positions;
}

vector<double> SimpleJoint::getMotorPositions(vector<double> jointPositions){
    vector<double> positions;
    
    for(int i = 0; i < jointPositions.size(); i++){
        positions[i] = jointPositions[i];
    }

    return positions;
}

vector<double> SimpleJoint::getMotorVelocities(vector<double> jointPositions){
    vector<double> setpoints;
    
    for(int i = 0; i < jointPositions.size(); i++){
        setpoints[i] = jointVelocites[i];
    }

    return setpoints;
}

void SimpleJoint::configVelocityHandshake(std::string jointTopicName, std::string motorTopicName){
    this->jointOutputSubscriber = this->n->subscribe<std_msgs::Float64>(jointTopicName + "/output", 1000, &SimpleJoint::handoffOutput, this);
    this->motorOutputPublisher = this->n->advertise<std_msgs::Float64>(motorTopicName + "/output", 1000);
    this->motorFeedbackSubscriber = this->n->subscribe<std_msgs::Float64>(jointTopicName + "/feeback", 1000, &SimpleJoint::handoffFeedback, this);
    this->jointFeedbackPublisher = this->n->advertise<std_msgs::Float64>(motorTopicName + "/feedback", 1000);
}

void SimpleJoint::handoffOutput(const std_msgs::Float64 msg){
    this->motorOutputPublisher.publish(msg);
}

void SimpleJoint::handoffFeedback(const std_msgs::Float64 msg){
    this->jointFeedbackPublisher.publish(msg);
}
