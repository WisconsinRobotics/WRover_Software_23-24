/**
 * @file ArmMotor.cpp
 * @author Ben Nowotny
 * @brief The implementation of the ArmMotor class
 * @date 2021-04-05
 */
#include "ArmMotor.hpp"

/// Allow for referencing the UInt32 message type easier
#define Std_UInt32 std_msgs::UInt32::ConstPtr&

/// The current COUNTS_PER_ROTATION is UINT32_MAX
unsigned int const ArmMotor::COUNTS_PER_ROTATION = UINT32_MAX;
/// The current encoders are absolute, and so can only perform one rotation
unsigned int const ArmMotor::ENCODER_BOUNDS[2] = {0, ArmMotor::COUNTS_PER_ROTATION};

template<class T> T ArmMotor::corrMod(T i, T j){
    // Stem i%j by j, which in modular arithmetic is the same as adding 0.
    return fmod(fmod(i,j)+j,j);
}

/// Currently consistent with the rad->enc equation as specified <a target="_blank" href="https://www.desmos.com/calculator/nwxtenccc6">here</a>.
unsigned int ArmMotor::radToEnc(double rads){
    return ArmMotor::COUNTS_PER_ROTATION*ArmMotor::corrMod(rads,2 * M_PI)/(2 * M_PI);
}

/// Currently consistent with the enc->rad equation as specified <a target="_blank" href="https://www.desmos.com/calculator/nwxtenccc6">here</a>.
float ArmMotor::getRads(){
    return ArmMotor::corrMod(this->getEncoderCounts() / ((float)ArmMotor::COUNTS_PER_ROTATION) * 2 * M_PI + M_PI, 2 * M_PI) - M_PI;
}

void ArmMotor::storeEncoderVals(const Std_UInt32 msg){
    // Store the message value in this ArmMotor's encoderVal variable
    this->encoderVal = msg->data;
}

/// controllerID is constrained between [0,3]
/// motorID is constrained between [0,1]
ArmMotor::ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle* n){
    // Check validity of WRoboclaw and motor IDs
    if(controllerID > 3) throw ((std::string)"Controller ID ") + std::to_string(controllerID) + "is only valid on [0,3]";
    if(motorID > 1) throw ((std::string)"Motor ID ") + std::to_string(motorID) + "is only valid on [0,1]";
    
    // Set the current name, controller, and motor ID
    this->motorName = motorName;
    this->controllerID = controllerID;
    this->motorID = motorID;
    // All motors start STOPped
    this->currState = MotorState::STOP;

    // Create the topic string prefix for WRoboclaw controllers
    std::string tpString = ((std::string)"/hsi/roboclaw/aux") + std::to_string(controllerID);

    // Create the appropriate encoder-reading and speed-publishing subscribers and advertisers, respectfully
    this->encRead = n->subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), 1000, &ArmMotor::storeEncoderVals, this);
    this->speedPub = n->advertise<std_msgs::Int16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), 1000);
}

unsigned int ArmMotor::getEncoderCounts(){
    return this->encoderVal;
}

void ArmMotor::runToTarget(unsigned int targetCounts, float power){
    this->runToTarget(targetCounts, power, false);
}

bool ArmMotor::hasReachedTarget(unsigned int targetCounts, unsigned int tolerance){
    // Compute the upper and lower bounds in the finite encoder space
    unsigned int lBound = ArmMotor::corrMod(targetCounts - tolerance, ArmMotor::ENCODER_BOUNDS[1]);
    unsigned int uBound = ArmMotor::corrMod(targetCounts + tolerance, ArmMotor::ENCODER_BOUNDS[1]);
    // If the computed lower bound is lower than the upper bound, perform the computation normally
    if(lBound < uBound)
        return this->getEncoderCounts() <= uBound && this->getEncoderCounts() >=lBound;
    // Otherwise, check if the value is outside either bound and negate the response
    else
        return this->getEncoderCounts() <= uBound || this->getEncoderCounts() >=lBound;
}

/// Current tolerance is &pm;0.1 degree w.r.t. the current number of counts per rotation
bool ArmMotor::hasReachedTarget(unsigned int targetCounts){
    return ArmMotor::hasReachedTarget(targetCounts, 1200000);
}

MotorState ArmMotor::getMotorState(){
    return this->currState;
}

/// This method auto-publishes the speed command to the WRoboclaws
void ArmMotor::setPower(float power){
    // Check the bounds of the parameter
    if(abs(power) > 1) throw ((std::string)"Power ") + std::to_string(power) + " is not on the interval [-1, 1]";

    // Set up and send the speed message
    this->powerMsg = new std_msgs::Int16();
    this->powerMsg->data = power * INT16_MAX;
    this->speedPub.publish(*(this->powerMsg));
    // Update the current motor state based on the power command input
    this->currState = power == 0.f ? MotorState::STOP : MotorState::MOVING;
}

void ArmMotor::runToTarget(unsigned int targetCounts, float power, bool block){
    // If we are not at our target...
    if(!this->hasReachedTarget(targetCounts)){
        // Set the power in the correct direction and continue running to the target
        long int direction = targetCounts - this->getEncoderCounts();
        power = abs(power) * (corrMod(direction, ((long int)ArmMotor::COUNTS_PER_ROTATION)) < corrMod(-direction, ((long int)ArmMotor::COUNTS_PER_ROTATION)) ? 1 : -1);
        this->setPower(power);
        this->currState = MotorState::RUN_TO_TARGET;
    // Otherwise, stop the motor
    }else{
        this->setPower(0.f);
        this->currState = MotorState::STOP;
    }
    // If this is a blocking call...
    if(block){
        // Wait until the motor has reached the target, then stop
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