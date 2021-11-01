/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "AbstractJoint.hpp"
using std::vector;

class DifferentialJoint : public AbstractJoint {
    public:
        ~DifferentialJoint();
        DifferentialJoint(ArmMotor* pitchMotor, ArmMotor* rollMotor, ros::NodeHandle* n);

        vector<double> getMotorPositions(vector<double> jointPositions);
        vector<double> getMotorVelocities(vector<double> jointVelocities);
        vector<double> getJointPositions(vector<double> motorPositions);

        void configVelocityHandshake(std::string jointTopicName, std::string motorTopicName);
        void handoffOutput(std_msgs::Float64);
        void handoffFeedback(std_msgs::Float64);

    private:
        double linearTransformation[2][2] = {{0.5, 0.5}, {1, -1}};
        
        ros::Subscriber jointOutputSubscriber;
        ros::Publisher motorOutputPublisher;

        ros::Subscriber motorFeedbackSubscriber;
        ros::Publisher jointFeedbackPublisher;
};

DifferentialJoint::DifferentialJoint(ArmMotor* pitchMotor, ArmMotor* rollMotor, ros::NodeHandle* n) : AbstractJoint(n) {
    this->numMotors = 2;
    this->motors[0] = pitchMotor;
    this->motors[1] = rollMotor;
}

vector<double> DifferentialJoint::getJointPositions(vector<double> motorPositions){
    vector<double> positions = *(new vector<double>()); // makes the red lines go away
    double motorPosition;

    for(int i = 0; i < motorPositions.size(); i++){
        positions[i] = motorPositions[0] * linearTransformation[0][i] + motorPositions[1] * linearTransformation[1][i];
    }

    return positions;
}

vector<double> DifferentialJoint::getMotorPositions(vector<double> jointPositions){
    vector<double> positions;
    
    for(int i = 0; i < jointPositions.size(); i++){
        positions[i] = jointPositions[i];
    }

    return positions;
}

vector<double> DifferentialJoint::getMotorVelocities(vector<double> jointPositions){
    vector<double> setpoints;
    
    for(int i = 0; i < jointPositions.size(); i++){
        setpoints[i] = jointVelocites[i];
    }

    return setpoints;
}

void DifferentialJoint::configVelocityHandshake(std::string jointTopicName, std::string motorTopicName){
    this->jointOutputSubscriber = this->n->subscribe<std_msgs::Float64>(jointTopicName + "/setpoint", 1000, &DifferentialJoint::handoffOutput, this);
    this->motorOutputPublisher = this->n->advertise<std_msgs::Float64>(motorTopicName + "/setpoint", 1000);
    this->motorFeedbackSubscriber = this->n->subscribe<std_msgs::Float64>(jointTopicName + "/feeback", 1000, &DifferentialJoint::handoffFeedback, this);
    this->jointFeedbackPublisher = this->n->advertise<std_msgs::Float64>(motorTopicName + "/feedback", 1000);
}

void DifferentialJoint::handoffOutput(const std_msgs::Float64 msg){
    this->motorOutputPublisher.publish(msg);
}

void DifferentialJoint::handoffFeedback(const std_msgs::Float64 msg){
    this->jointFeedbackPublisher.publish(msg);
}
