#include "ArmMotor.hpp"

#define Std_UInt32 std_msgs::UInt32::ConstPtr&

unsigned int const ArmMotor::COUNTS_PER_ROTATION = UINT32_MAX;
unsigned int const ArmMotor::ENCODER_BOUNDS[2] = {0, ArmMotor::COUNTS_PER_ROTATION};

template<class T> T ArmMotor::corrMod(T i, T j){
    int dir = -((i-j>0)-(i-j<0));
    while(i<0||i>=j)i+=dir*j;
    return i;
}

unsigned int ArmMotor::radToEnc(double rads){
    return ArmMotor::COUNTS_PER_ROTATION*ArmMotor::corrMod(rads,2 * M_PI)/(2 * M_PI);
}

float ArmMotor::getRads(){
    return ArmMotor::corrMod(this->getEncoderCounts() / ((float)ArmMotor::COUNTS_PER_ROTATION) * 2 * M_PI + M_PI, 2 * M_PI) - M_PI;
}

void ArmMotor::storeEncoderVals(const Std_UInt32 msg){
    // std::cout<<"I ran "/*mtr:"<<this->getMotorName()<<*/" val: "<<msg->data<<std::endl;
    this->encoderVal = msg->data;
    // std::cout<<"value stored: "<<this->encoderVal<<std::endl;
    // std::cout<<"value in method: "<<this->getEncoderCounts()<<std::endl;
}


ArmMotor::ArmMotor(){}

ArmMotor::ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle* n){
    if(controllerID > 3) throw ((std::string)"Controller ID ") + std::to_string(controllerID) + "is only valid on [0,3]";
    if(motorID > 1) throw ((std::string)"Motor ID ") + std::to_string(motorID) + "is only valid on [0,1]";
    
    this->motorName = motorName;
    this->controllerID = controllerID;
    this->motorID = motorID;
    this->currState = MotorState::STOP;

    std::string tpString = ((std::string)"/hsi/roboclaw/aux") + std::to_string(controllerID);

    this->encRead = n->subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), 1000, &ArmMotor::storeEncoderVals, this);
    this->speedPub = n->advertise<std_msgs::Int16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), 1000);
}

// ~ArmMotor();

unsigned int ArmMotor::getEncoderCounts(){
    return this->encoderVal;
}

// void resetEncoder();

void ArmMotor::runToTarget(unsigned int targetCounts, float power){
    this->runToTarget(targetCounts, power, false);
}

bool ArmMotor::hasReachedTarget(unsigned int targetCounts, unsigned int tolerance){
    unsigned int lBound = ((targetCounts - tolerance) % ArmMotor::ENCODER_BOUNDS[1] < 0 ? 2 * targetCounts - tolerance : targetCounts - tolerance) % ArmMotor::ENCODER_BOUNDS[1];
    unsigned int uBound = (targetCounts + tolerance) % ArmMotor::ENCODER_BOUNDS[1];
    if(lBound < uBound)
        return this->getEncoderCounts() <= uBound && this->getEncoderCounts() >=lBound;
    else
        return this->getEncoderCounts() <= uBound || this->getEncoderCounts() >=lBound;
}

bool ArmMotor::hasReachedTarget(unsigned int targetCounts){
    return ArmMotor::hasReachedTarget(targetCounts, 1200000);
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

void ArmMotor::runToTarget(unsigned int targetCounts, float power, bool block){
    if(this->getMotorState() != MotorState::RUN_TO_TARGET && !this->hasReachedTarget(targetCounts)){
        long int direction = targetCounts - this->getEncoderCounts();
        power = abs(power) * (corrMod(direction, ((long int)ArmMotor::COUNTS_PER_ROTATION)) < corrMod(-direction, ((long int)ArmMotor::COUNTS_PER_ROTATION)) ? 1 : -1);
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

std::string ArmMotor::getMotorName(){
    return this->motorName;
}

float ArmMotor::getPower(){
    return ((float)this->powerMsg->data)/INT16_MAX;
}