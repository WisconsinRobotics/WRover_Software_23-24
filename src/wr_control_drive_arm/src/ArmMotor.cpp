/**
 * @file ArmMotor.cpp
 * @author Ben Nowotny
 * @brief The implementation of the ArmMotor class
 * @date 2021-04-05
 */
#include "ArmMotor.hpp"

/// Allow for referencing the UInt32 message type easier
typedef std_msgs::UInt32::ConstPtr Std_UInt32;
typedef std_msgs::Float64::ConstPtr Std_Float64;
typedef std_msgs::Bool::ConstPtr Std_Bool;

constexpr float STALL_THRESHOLD_TIME = 0.5;

double ArmMotor::corrMod(double i, double j){
    // Stem i%j by j, which in modular arithmetic is the same as adding 0.
    return std::fmod(std::fmod(std::abs(j)*i/j,std::abs(j))+j,std::abs(j));
}

/// Currently consistent with the rad->enc equation as specified <a target="_blank" href="https://www.desmos.com/calculator/nwxtenccc6">here</a>.
uint32_t ArmMotor::radToEnc(double rads) const {
    return this->COUNTS_PER_ROTATION * ArmMotor::corrMod(rads, 2 * M_PI)/(2 * M_PI) + this->ENCODER_OFFSET;
}

double ArmMotor::encToRad(uint32_t enc) const {
    return ArmMotor::corrMod(static_cast<double>(enc - this->ENCODER_OFFSET) / static_cast<double>(this->COUNTS_PER_ROTATION) * 2 * M_PI + M_PI, 2 * M_PI) - M_PI;
}

/// Currently consistent with the enc->rad equation as specified <a target="_blank" href="https://www.desmos.com/calculator/nwxtenccc6">here</a>.
double ArmMotor::getRads() const{
    return ArmMotor::encToRad(this->getEncoderCounts());
}

void ArmMotor::storeEncoderVals(const Std_UInt32 &msg){
    // Store the message value in this ArmMotor's encoderVal variable
    this->encoderVal = msg->data;
    // Send feedback
    std_msgs::Float64 feedbackMsg;
    feedbackMsg.data = ArmMotor::encToRad(msg->data);
    this->feedbackPub.publish(feedbackMsg);
}

void ArmMotor::redirectPowerOutput(const Std_Float64 &msg){
    // Set the speed to be the contained data
    this->setPower(msg->data);
}

void ArmMotor::storeStallStatus(const Std_Bool &msg) {
    this->isStall = static_cast<bool>(msg->data);
}

/// controllerID is constrained between [0,3]
/// motorID is constrained between [0,1]
ArmMotor::ArmMotor(
    const std::string &motor_name,
    unsigned int controllerID,
    unsigned int motorID,
    int64_t countsPerRotation,
    int64_t offset,
    ros::NodeHandle &n
) : COUNTS_PER_ROTATION{countsPerRotation}, 
    ENCODER_BOUNDS{0, std::abs(countsPerRotation)}, 
    ENCODER_OFFSET{offset}, 
    motorName{motor_name}, 
    controllerID{controllerID}, 
    motorID{motorID}, 
    currState{MotorState::STOP},
    encoderVal{static_cast<uint32_t>(offset)} {
    
    // Check validity of WRoboclaw and motor IDs
    if(controllerID > 3) throw ((std::string)"Controller ID ") + std::to_string(controllerID) + "is only valid on [0,3]";
    if(motorID > 1) throw ((std::string)"Motor ID ") + std::to_string(motorID) + "is only valid on [0,1]";

    // Create the topic string prefix for WRoboclaw controllers
    std::string tpString = ((std::string)"/hsi/roboclaw/aux") + std::to_string(controllerID);
    std::string controlString = "/control/arm/" + std::to_string(controllerID) + std::to_string(motorID);

    // Create the appropriate encoder-reading and speed-publishing subscribers and advertisers, respectfully
    this->encRead = n.subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), ArmMotor::MESSAGE_CACHE_SIZE, &ArmMotor::storeEncoderVals, this);
    this->speedPub = n.advertise<std_msgs::Int16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), ArmMotor::MESSAGE_CACHE_SIZE);
    this->targetPub = n.advertise<std_msgs::Float64>(controlString + "/setpoint", ArmMotor::MESSAGE_CACHE_SIZE);
    this->feedbackPub = n.advertise<std_msgs::Float64>(controlString + "/feedback", ArmMotor::MESSAGE_CACHE_SIZE);
    this->outputRead = n.subscribe(controlString + "/output", ArmMotor::MESSAGE_CACHE_SIZE, &ArmMotor::redirectPowerOutput, this);
    this->stallRead = n.subscribe(tpString + "/curr/over_lim/" + (motorID == 0 ? "left" : "right"), ArmMotor::MESSAGE_CACHE_SIZE, &ArmMotor::storeStallStatus, this);
}

uint32_t ArmMotor::getEncoderCounts() const{
    return this->encoderVal;
}

void ArmMotor::runToTarget(uint32_t targetCounts, float power){
    std::cout << "run to enc: " << targetCounts << std::endl;
    this->runToTarget(targetCounts, power, false);
}

bool ArmMotor::hasReachedTarget(uint32_t targetCounts, uint32_t tolerance) const {
    // Compute the upper and lower bounds in the finite encoder space
    uint32_t lBound = ArmMotor::corrMod(static_cast<double>(targetCounts - tolerance), static_cast<double>(ArmMotor::ENCODER_BOUNDS[1]));
    uint32_t uBound = ArmMotor::corrMod(static_cast<double>(targetCounts + tolerance), static_cast<double>(ArmMotor::ENCODER_BOUNDS[1]));
    // If the computed lower bound is lower than the upper bound, perform the computation normally
    if(lBound < uBound)
        return this->getEncoderCounts() <= uBound && this->getEncoderCounts() >=lBound;
    // Otherwise, check if the value is outside either bound and negate the response
    return this->getEncoderCounts() <= uBound || this->getEncoderCounts() >=lBound;
}

/// Current tolerance is &pm;0.1 degree w.r.t. the current number of counts per rotation
bool ArmMotor::hasReachedTarget(uint32_t targetCounts) const {
    return ArmMotor::hasReachedTarget(targetCounts, ArmMotor::TOLERANCE_RATIO * static_cast<double>(std::abs(this->COUNTS_PER_ROTATION)));
}

MotorState ArmMotor::getMotorState() const {
    return this->currState;
}

/// This method auto-publishes the speed command to the WRoboclaws
void ArmMotor::setPower(float power){
    // Check the bounds of the parameter
    if(abs(power) > 1) throw ((std::string)"Power ") + std::to_string(power) + " is not on the interval [-1, 1]";

    // Set up and send the speed message
    this->powerMsg.data = power * INT16_MAX;
    this->speedPub.publish(this->powerMsg);
    // Update the cur.nt motor state based on the power command input
    this->currState = power == 0.F ? MotorState::STOP : MotorState::MOVING;
}

bool ArmMotor::runToTarget(uint32_t targetCounts, float power, bool block){
    // Checks for stall
    if (this->isStall) {
        if ((ros::Time::now() - begin).toSec() >= STALL_THRESHOLD_TIME) {
            this->setPower(0.0F);
            this->currState = MotorState::STOP;
            return false;
        }
    } else {
        begin = ros::Time::now();
    }
    
    // If we are not at our target...
    if(!this->hasReachedTarget(targetCounts)){
        // Set the power in the correct direction and continue running to the target
        std_msgs::Float64 setpointMsg;
        setpointMsg.data = ArmMotor::encToRad(targetCounts);
        this->targetPub.publish(setpointMsg);

        // long int direction = targetCounts - this->getEncoderCounts();
        // power = abs(power) * (corrMod(direction, ((long int)this->COUNTS_PER_ROTATION)) < corrMod(-direction, ((long int)this->COUNTS_PER_ROTATION)) ? 1 : -1);
        // this->setPower(power);

        this->currState = MotorState::RUN_TO_TARGET;
    // Otherwise, stop the motor
    } else {
        this->setPower(0.F);
        this->currState = MotorState::STOP;
    }
    // If this is a blocking call...
    if(block){
        // Wait until the motor has reached the target, then stop
        while(!this->hasReachedTarget(targetCounts));
        this->setPower(0.F);
        this->currState = MotorState::STOP;
    }
    return true;
}

bool ArmMotor::runToTarget(double rads, float power){
    std::cout << "run to target: " << rads << ":" << this->radToEnc(rads) << std::endl;
    
    return runToTarget(this->radToEnc(rads), power, false);
}

std::string ArmMotor::getMotorName() const {
    return this->motorName;
}

unsigned int ArmMotor::getMotorID() const {
    return this->motorID;
}

unsigned int ArmMotor::getControllerID() const {
    return this->controllerID;
}

float ArmMotor::getPower() const {
    return ((float)this->powerMsg.data)/INT16_MAX;
}