/**
 * @file AbstractJoint.hpp
 * @author Nichols Underwood
 * @brief ablskjlfkejfs
 * @date 2021-10-25
 */

#include "DifferentialJoint.hpp"
#include <memory>
using std::vector;

constexpr std::array<std::array<double, 2>, 2> DifferentialJoint::MOTOR_TO_JOINT_MATRIX;
constexpr std::array<std::array<double, 2>, 2> DifferentialJoint::JOINT_TO_MOTOR_MATRIX;

DifferentialJoint::DifferentialJoint(std::unique_ptr<ArmMotor> leftMotor, std::unique_ptr<ArmMotor> rightMotor, ros::NodeHandle &n,
                                     const std::string &pitchTopicName, const std::string &rollTopicName,
                                     const std::string &leftTopicName, const std::string &rightTopicName) 
                                     : AbstractJoint(n, DEGREES_OF_FREEDOM) {
    this->motors.at(0) = {std::move(leftMotor), 0, 0, "", "", false};
    this->motors.at(1) = {std::move(rightMotor), 0, 0, "", "", false};

    this->pitchOutputSubscriber = n.subscribe<std_msgs::Float64>(pitchTopicName + "/output", MESSAGE_CACHE_SIZE, &DifferentialJoint::handoffPitchOutput, this);
    this->rollOutputSubscriber = n.subscribe<std_msgs::Float64>(rollTopicName + "/output", MESSAGE_CACHE_SIZE, &DifferentialJoint::handoffRollOutput, this);
    this->leftOutputPublisher = n.advertise<std_msgs::Float64>(leftTopicName + "/output", MESSAGE_CACHE_SIZE);
    this->rightOutputPublisher = n.advertise<std_msgs::Float64>(rightTopicName + "/output", MESSAGE_CACHE_SIZE);
    this->leftFeedbackSubscriber = n.subscribe<std_msgs::Float64>(leftTopicName + "/feeback", MESSAGE_CACHE_SIZE, &DifferentialJoint::handoffLeftFeedback, this);
    this->rightFeedbackSubscriber = n.subscribe<std_msgs::Float64>(rightTopicName + "/feeback", MESSAGE_CACHE_SIZE, &DifferentialJoint::handoffRightFeedback, this);
    this->pitchFeedbackPublisher = n.advertise<std_msgs::Float64>(pitchTopicName + "/feedback", MESSAGE_CACHE_SIZE);
    this->rollFeedbackPublisher = n.advertise<std_msgs::Float64>(rollTopicName + "/feedback", MESSAGE_CACHE_SIZE);
}

auto DifferentialJoint::getJointPositions(const vector<double> &motorPositions) -> vector<double>{
    // vector<double> positions;
    // target->reserve(2);
    
    double pitch = motorPositions.at(0) * MOTOR_TO_JOINT_MATRIX.at(0).at(0) + motorPositions.at(1) * MOTOR_TO_JOINT_MATRIX.at(0).at(1);
    double roll = motorPositions.at(0) * MOTOR_TO_JOINT_MATRIX.at(1).at(0) + motorPositions.at(1) * MOTOR_TO_JOINT_MATRIX.at(1).at(1);

    //TODO: Why not return a new vector?  Wouldn't this constant resizing be more inefficient than just making/returning a new vector via copy ellision?
    return {pitch, roll};

    // return positions;
}

auto DifferentialJoint::getMotorPositions(const vector<double> &jointPositions) -> vector<double>{
    
    // std::unique_ptr<vector<double>> positions = std::make_unique<vector<double>>(2);
    // target->reserve(2);

    double left = jointPositions.at(0) * JOINT_TO_MOTOR_MATRIX.at(0).at(0) + jointPositions.at(1) * JOINT_TO_MOTOR_MATRIX.at(0).at(1);
    double right = jointPositions.at(0) * JOINT_TO_MOTOR_MATRIX.at(1).at(0) + jointPositions.at(1) * JOINT_TO_MOTOR_MATRIX.at(1).at(1);

    return {left, right};

    // return std::move(positions);
}

auto DifferentialJoint::getMotorVelocities(const vector<double> &jointPositions) -> vector<double>{
    return getMotorPositions(jointPositions); //deritivate of linear transformation is itself
}

void DifferentialJoint::handoffPitchOutput(const std_msgs::Float64::ConstPtr &msg){
    this->cachedPitchOutput = msg->data;
    this->hasNewPitchOutput = true;
    handOffAllOutput();
}
void DifferentialJoint::handoffRollOutput(const std_msgs::Float64::ConstPtr &msg){
    this->cachedRollOutput = msg->data;
    this->hasNewRollOutput = true;
    handOffAllOutput();
}

// TODO: Do these ouputs need to be synced?  We're just referring to the motors individually, maybe we can transform and send the data right away
void DifferentialJoint::handOffAllOutput(){
    if(!this->hasNewPitchOutput || !this->hasNewRollOutput){ return; }

    vector<double> outputs;
    outputs.resize(2);
    outputs.at(0) = this->cachedPitchOutput;
    outputs.at(1) = this->cachedRollOutput;
    //TODO: Still confused as to the purpose of this family of functions
    outputs = getMotorVelocities(outputs); 

    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    msg1.data = outputs.at(0);
    msg2.data = outputs.at(1);
    pitchFeedbackPublisher.publish(msg1);
    rollFeedbackPublisher.publish(msg2);

    leftOutputPublisher.publish(msg1);
    rightOutputPublisher.publish(msg2);

    this->hasNewPitchOutput = false;
    this->hasNewRollOutput = false;
    
}

void DifferentialJoint::handoffRightFeedback(const std_msgs::Float64::ConstPtr &msg){
    this->cachedRightFeedback = msg->data;
    this->hasNewRightFeedback = true;
    handOffAllFeedback();
}
void DifferentialJoint::handoffLeftFeedback(const std_msgs::Float64::ConstPtr &msg){
    this->cachedLeftFeedback = msg->data;
    this->hasNewLeftFeedback = true;
    handOffAllFeedback();
}

void DifferentialJoint::handOffAllFeedback(){
    if(!this->hasNewLeftFeedback || !this->hasNewRightFeedback){ return; }

    vector<double> outputs;
    outputs.at(0) = this->cachedLeftFeedback;
    outputs.at(1) = this->cachedRightFeedback;
    outputs = getMotorVelocities(outputs); 

    std_msgs::Float64 msg1;
    std_msgs::Float64 msg2;
    msg1.data = outputs.at(0);
    msg2.data = outputs.at(1);
    pitchFeedbackPublisher.publish(msg1);
    rollFeedbackPublisher.publish(msg2);

    this->hasNewLeftFeedback = false;
    this->hasNewRightFeedback = false;
    
}
