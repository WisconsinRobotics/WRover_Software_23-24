#include "ArmMotor.hpp"

#define Std_UInt32 std_msgs::UInt32::ConstPtr&

static const unsigned long int COUNTS_PER_ROTATION = UINT_FAST32_MAX;

int ArmMotor::radToEnc(float rads){
    return COUNTS_PER_ROTATION * rads / (2 * M_PI);
}

void ArmMotor::storeEncoderVals(const Std_UInt32 msg){
    // std::cout<<"I ran "/*mtr:"<<this->getMotorName()<<*/" val: "<<msg->data<<std::endl;
    this->encoderVal = msg->data;
    // std::cout<<"value stored: "<<this->encoderVal<<std::endl;
    // std::cout<<"value in method: "<<this->getEncoderCounts()<<std::endl;
}


ArmMotor::ArmMotor(){}

ArmMotor::ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle n){
    if(controllerID > 3) throw ((std::string)"Controller ID ") + std::to_string(controllerID) + "is not valid on [0,3]";
    if(motorID > 1) throw ((std::string)"Motor ID ") + std::to_string(motorID) + "is not valid on [0,1]";
    
    this->motorName = motorName;
    this->controllerID = controllerID;
    this->motorID = motorID;
    this->currState = MotorState::STOP;

    std::string tpString = ((std::string)"/hsi/roboclaw/aux") + std::to_string(controllerID);

    this->encRead = n.subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), 1000, &ArmMotor::storeEncoderVals, this);
    this->speedPub = n.advertise<std_msgs::Int16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), 1000);
}

// ~ArmMotor();

int ArmMotor::getEncoderCounts(){
    return this->encoderVal;
}

// void resetEncoder();

void ArmMotor::runToTarget(int targetCounts, float power){
    this->runToTarget(targetCounts, power, false);
}

bool ArmMotor::hasReachedTarget(int targetCounts){
    return abs(targetCounts - this->getEncoderCounts()) < 10;
}

MotorState ArmMotor::getMotorState(){
    return this->currState;
}

void ArmMotor::setPower(float power){
    if(abs(power) > 1) throw ((std::string)"Power ") + std::to_string(power) + " is not on the interval [-1, 1]";

    this->powerMsg = new std_msgs::Int16();
    this->powerMsg->data = power * INT16_MAX;
    this->speedPub.publish(*(this->powerMsg));
    this->currState = power == 0.f ? MotorState::STOP : MotorState::MOVING;
}

void ArmMotor::runToTarget(int targetCounts, float power, bool block){
    if(this->getMotorState() != MotorState::RUN_TO_TARGET){
        power = abs(power) * abs(targetCounts - this->getEncoderCounts())/(targetCounts - this->getEncoderCounts());
        this->setPower(power);
        this->currState = MotorState::RUN_TO_TARGET;
    }
    if(this->hasReachedTarget(targetCounts)){
        this->setPower(0.f);
        this->currState = MotorState::STOP;
    }
    if(block){
        while(!this->hasReachedTarget(targetCounts));
        this->setPower(0.f);
        this->currState = MotorState::STOP;
    }
}

void ArmMotor::runToTarget(double rads, float power){
    runToTarget(this->radToEnc(rads), power, false);
}

double ArmMotor::getRads(){
    return this->getEncoderCounts() * 2 * M_PI / COUNTS_PER_ROTATION;
}

std::string ArmMotor::getMotorName(){
    return this->motorName;
}

float ArmMotor::getPower(){
    return ((float)this->powerMsg->data)/INT16_MAX;
}