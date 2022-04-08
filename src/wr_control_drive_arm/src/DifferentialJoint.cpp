/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "DifferentialJoint.hpp"
#include <memory>
using std::vector;

DifferentialJoint::DifferentialJoint(ArmMotor* leftMotor, ArmMotor* rightMotor, ros::NodeHandle* n) : AbstractJoint(n, 2) {
    this->motors.push_back(leftMotor);
    this->motors.push_back(rightMotor);
}

void DifferentialJoint::getJointPositions(vector<double> &motorPositions, vector<double> &target){
    // vector<double> positions;
    // target->reserve(2);
    
    double pitch = motorPositions[0] * this->motorToJointMatrix[0][0] + motorPositions[1]*this->motorToJointMatrix[0][1];
    double roll = motorPositions[1] * this->motorToJointMatrix[1][0] + motorPositions[1]*this->motorToJointMatrix[1][1];

    target.push_back(pitch);
    target.push_back(roll);

    // return positions;
}

void DifferentialJoint::getMotorPositions(vector<double> &jointPositions, vector<double> &target){
    
    // std::unique_ptr<vector<double>> positions = std::make_unique<vector<double>>(2);
    // target->reserve(2);

    double left = jointPositions[0] * this->jointToMotorMatrix[0][0] + jointPositions[1]*this->jointToMotorMatrix[0][1];
    double right = jointPositions[1] * this->jointToMotorMatrix[1][0] + jointPositions[1]*this->jointToMotorMatrix[1][1];

    target.push_back(left);
    target.push_back(right);

    // return std::move(positions);
}

void DifferentialJoint::getMotorVelocities(vector<double> &jointPositions, vector<double> &target){
    return getMotorPositions(jointPositions, target); //deritivate of linear transformation is itself
}

void DifferentialJoint::configVelocityHandshake(std::string pitchTopicName, std::string rollTopicName, std::string leftTopicName, std::string rightTopicName){
    this->pitchOutputSubscriber = this->n->subscribe<std_msgs::Float64>(pitchTopicName + "/output", 1000, &DifferentialJoint::handoffPitchOutput, this);
    this->rollOutputSubscriber = this->n->subscribe<std_msgs::Float64>(rollTopicName + "/output", 1000, &DifferentialJoint::handoffRollOutput, this);
    this->leftOutputPublisher = this->n->advertise<std_msgs::Float64>(leftTopicName + "/output", 1000);
    this->rightOutputPublisher = this->n->advertise<std_msgs::Float64>(rightTopicName + "/output", 1000);
    this->leftFeedbackSubscriber = this->n->subscribe<std_msgs::Float64>(leftTopicName + "/feeback", 1000, &DifferentialJoint::handoffLeftFeedback, this);
    this->rightFeedbackSubscriber = this->n->subscribe<std_msgs::Float64>(rightTopicName + "/feeback", 1000, &DifferentialJoint::handoffRightFeedback, this);
    this->pitchFeedbackPublisher = this->n->advertise<std_msgs::Float64>(pitchTopicName + "/feedback", 1000);
    this->rollFeedbackPublisher = this->n->advertise<std_msgs::Float64>(rollTopicName + "/feedback", 1000);
}

void DifferentialJoint::handoffPitchOutput(const std_msgs::Float64 msg){
    this->cachedPitchOutput = msg.data;
    this->hasNewPitchOutput = true;
    handOffAllOutput();
}
void DifferentialJoint::handoffRollOutput(const std_msgs::Float64 msg){
    this->cachedRollOutput = msg.data;
    this->hasNewRollOutput = true;
    handOffAllOutput();
}

void DifferentialJoint::handOffAllOutput(){
    if(!this->hasNewPitchOutput || !this->hasNewRollOutput){ return; }

    vector<double> outputs;
    outputs[0] = this->cachedPitchOutput;
    outputs[1] = this->cachedRollOutput;
    getMotorVelocities(outputs, outputs); 

    std_msgs::Float64 msg1 = std_msgs::Float64();
    std_msgs::Float64 msg2 = std_msgs::Float64();
    msg1.data = outputs[0];
    msg2.data = outputs[1];
    pitchFeedbackPublisher.publish(msg1);
    rollFeedbackPublisher.publish(msg2);

    leftOutputPublisher.publish(msg1);
    rightOutputPublisher.publish(msg2);

    this->hasNewPitchOutput = false;
    this->hasNewRollOutput = false;
    
}

void DifferentialJoint::handoffRightFeedback(const std_msgs::Float64 msg){
    this->cachedRightFeedback = msg.data;
    this->hasNewRightFeedback = true;
    handOffAllFeedback();
}
void DifferentialJoint::handoffLeftFeedback(const std_msgs::Float64 msg){
    this->cachedLeftFeedback = msg.data;
    this->hasNewLeftFeedback = true;
    handOffAllFeedback();
}

void DifferentialJoint::handOffAllFeedback(){
    if(!this->hasNewLeftFeedback || !this->hasNewRightFeedback){ return; }

    vector<double> outputs;
    outputs[0] = this->cachedLeftFeedback;
    outputs[1] = this->cachedRightFeedback;
    getMotorVelocities(outputs, outputs); 

    std_msgs::Float64 msg1 = std_msgs::Float64();
    std_msgs::Float64 msg2 = std_msgs::Float64();
    msg1.data = outputs[0];
    msg2.data = outputs[1];
    pitchFeedbackPublisher.publish(msg1);
    rollFeedbackPublisher.publish(msg2);

    this->hasNewLeftFeedback = false;
    this->hasNewRightFeedback = false;
    
}
