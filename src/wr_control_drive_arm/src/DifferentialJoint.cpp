/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "DifferentialJoint.hpp"
using std::vector;

DifferentialJoint::DifferentialJoint(ArmMotor* leftMotor, ArmMotor* rightMotor, ros::NodeHandle* n) : AbstractJoint(n) {
    this->numMotors = 2;
    this->motors[0] = leftMotor;
    this->motors[1] = rightMotor;
}

vector<double> DifferentialJoint::getJointPositions(vector<double> motorPositions){
    vector<double> positions;
    
    double pitch = motorPositions[0] * this->motorToJointMatrix[0][0] + motorPositions[1]*this->motorToJointMatrix[0][1];
    double roll = motorPositions[1] * this->motorToJointMatrix[1][0] + motorPositions[1]*this->motorToJointMatrix[1][1];

    positions[0] = pitch;
    positions[1] = roll;

    return positions;
}

vector<double> DifferentialJoint::getMotorPositions(vector<double> jointPositions){
    
    vector<double> positions;
    
    double left = jointPositions[0] * this->jointToMotorMatrix[0][0] + jointPositions[1]*this->jointToMotorMatrix[0][1];
    double right = jointPositions[1] * this->jointToMotorMatrix[1][0] + jointPositions[1]*this->jointToMotorMatrix[1][1];

    positions[0] = left;
    positions[1] = right;

    return positions;
}

vector<double> DifferentialJoint::getMotorVelocities(vector<double> jointPositions){
    return getMotorPositions(jointPositions); //deritivate of linear transformation is itself
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
    outputs = getMotorVelocities(outputs); 

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
    outputs = getMotorVelocities(outputs); 

    std_msgs::Float64 msg1 = std_msgs::Float64();
    std_msgs::Float64 msg2 = std_msgs::Float64();
    msg1.data = outputs[0];
    msg2.data = outputs[1];
    pitchFeedbackPublisher.publish(msg1);
    rollFeedbackPublisher.publish(msg2);

    this->hasNewLeftFeedback = false;
    this->hasNewRightFeedback = false;
    
}
