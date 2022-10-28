/**
 * @file ArmMotor.cpp
 * @author Ben Nowotny
 * @brief The implementation of the ArmMotor class
 * @date 2021-04-05
 */
#include "ArmMotor.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <sstream>

/// Allow for referencing the UInt32 message type easier
using Std_UInt32 = std_msgs::UInt32::ConstPtr;
using Std_Float64 = std_msgs::Float64::ConstPtr;
using Std_Bool = std_msgs::Bool::ConstPtr;

auto ArmMotor::corrMod(double i, double j) -> double {
    // Stem i%j by j, which in modular arithmetic is the same as adding 0.
    return std::fmod(std::fmod(std::abs(j)*i/j,std::abs(j))+j,std::abs(j));
}

/// Currently consistent with the rad->enc equation as specified <a target="_blank" href="https://www.desmos.com/calculator/nwxtenccc6">here</a>.
auto ArmMotor::radToEnc(double rads) const -> uint32_t{
    double remappedEnc = ArmMotor::corrMod(rads, 2 * M_PI)/(2 * M_PI);
    return ArmMotor::corrMod((this->COUNTS_PER_ROTATION * remappedEnc) + this->ENCODER_OFFSET, abs(this->COUNTS_PER_ROTATION));
}

auto ArmMotor::encToRad(uint32_t enc) const -> double{
    double remappedEnc = ArmMotor::corrMod(static_cast<double>(enc - this->ENCODER_OFFSET), abs(static_cast<double>(this->COUNTS_PER_ROTATION))) / this->COUNTS_PER_ROTATION;
    return ArmMotor::corrMod( remappedEnc * 2 * M_PI + M_PI, 2 * M_PI) - M_PI;
}

/// Currently consistent with the enc->rad equation as specified <a target="_blank" href="https://www.desmos.com/calculator/nwxtenccc6">here</a>.
auto ArmMotor::getRads() const -> double{
    return ArmMotor::encToRad(this->getEncoderCounts());
}

void ArmMotor::storeEncoderVals(const Std_UInt32 &msg){
    // Store the message value in this ArmMotor's encoderVal variable
    this->encoderVal = msg->data;
    
    // Send feedback
    std_msgs::Float64 feedbackMsg;
    feedbackMsg.data = ArmMotor::encToRad(msg->data);
    this->feedbackPub.publish(feedbackMsg);

    if(this->currState == MotorState::RUN_TO_TARGET){
        std::cout << "[2] motor " << motorName << " position " << (hasReachedTarget(this->target) ? "at target " : "not at target ") << this->target << ":" << this->encoderVal << std::endl;
        if(hasReachedTarget(this->target)){
            std::cout << "[1] stop motor" << std::endl;
            this->setPower(0.F, MotorState::STOP);
        }
    }
}

void ArmMotor::redirectPowerOutput(const Std_Float64 &msg){
    // Set the speed to be the contained data
    if(abs(msg->data) > 1) std::cout << "Received power " << msg->data << " from PID for motor " << motorName << std::endl;
    if(this->getMotorState() == MotorState::RUN_TO_TARGET) this->setPower(static_cast<float>(msg->data) * this->maxPower, MotorState::RUN_TO_TARGET);
}

void ArmMotor::storeStallStatus(const Std_Bool &msg) {
    if (static_cast<bool>(msg->data)) {
        if ((ros::Time::now() - beginStallTime).toSec() >= STALL_THRESHOLD_TIME) {
            std::cout << "over lim" << std::endl;
            this->setPower(0.0F, MotorState::STALLING);
        }
    } else {
        beginStallTime = ros::Time::now();
    }
}

/// controllerID is constrained between [0,3]
/// motorID is constrained between [0,1]
ArmMotor::ArmMotor(
    std::string motor_name,
    unsigned int controllerID,
    unsigned int motorID,
    int64_t countsPerRotation,
    int64_t offset,
    ros::NodeHandle &n
) : COUNTS_PER_ROTATION{countsPerRotation}, 
    ENCODER_BOUNDS{0, std::abs(countsPerRotation)}, 
    ENCODER_OFFSET{offset}, 
    motorName{std::move(motor_name)}, 
    controllerID{controllerID}, 
    motorID{motorID}, 
    currState{MotorState::STOP},
    power{0.F},
    maxPower{0.F},
    encoderVal{static_cast<uint32_t>(offset)} {
    
    // Check validity of WRoboclaw and motor IDs
    if(controllerID > 3) throw std::invalid_argument{std::string{"Controller ID "} + std::to_string(controllerID) + "is only valid on [0,3]"};
    if(motorID > 1) throw std::invalid_argument{std::string{"Motor ID "} + std::to_string(motorID) + " is only valid on [0,1]"};

    // Create the topic string prefix for WRoboclaw controllers
    std::string tpString = std::string{"/hsi/roboclaw/aux"} + std::to_string(controllerID);
    std::string controlString = "/control/arm/" + std::to_string(controllerID) + std::to_string(motorID);

    // Create the appropriate encoder-reading and speed-publishing subscribers and advertisers, respectfully
    this->encRead = n.subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), ArmMotor::MESSAGE_CACHE_SIZE, &ArmMotor::storeEncoderVals, this);
    this->speedPub = n.advertise<std_msgs::Int16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), ArmMotor::MESSAGE_CACHE_SIZE);
    this->targetPub = n.advertise<std_msgs::Float64>(controlString + "/setpoint", ArmMotor::MESSAGE_CACHE_SIZE);
    this->feedbackPub = n.advertise<std_msgs::Float64>(controlString + "/feedback", ArmMotor::MESSAGE_CACHE_SIZE);
    this->outputRead = n.subscribe(controlString + "/output", ArmMotor::MESSAGE_CACHE_SIZE, &ArmMotor::redirectPowerOutput, this);
    this->stallRead = n.subscribe(tpString + "/curr/over_lim/" + (motorID == 0 ? "left" : "right"), ArmMotor::MESSAGE_CACHE_SIZE, &ArmMotor::storeStallStatus, this);

    std::cout << this->motorName << ": " << this->COUNTS_PER_ROTATION << std::endl;
}

auto ArmMotor::getEncoderCounts() const -> uint32_t{
    return this->encoderVal;
}

void ArmMotor::runToTarget(uint32_t targetCounts, float power){
    std::cout << "run to enc: " << targetCounts << std::endl;
    this->runToTarget(targetCounts, power, false);
}

auto ArmMotor::hasReachedTarget(uint32_t targetCounts, uint32_t tolerance) const -> bool{
    std::cout << "TOLERANCE: " << tolerance << std::endl;
    // Compute the upper and lower bounds in the finite encoder space
    int32_t lBound = ArmMotor::corrMod(static_cast<double>(targetCounts - tolerance), static_cast<double>(ArmMotor::ENCODER_BOUNDS[1]));
    int32_t uBound = ArmMotor::corrMod(static_cast<double>(targetCounts + tolerance), static_cast<double>(ArmMotor::ENCODER_BOUNDS[1]));
    std::cout << "LBOUND: " << (lBound) << " UBOUND: " << (uBound) << std::endl;
    auto position = ArmMotor::corrMod(getEncoderCounts(), ENCODER_BOUNDS[1]);
    std::cout << "POSITION RAW: " << encToRad(getEncoderCounts()) << "/" << getEncoderCounts() << std::endl;
    std::cout << "POSITION: " << encToRad(position) << "/" << position << std::endl;
    // If the computed lower bound is lower than the upper bound, perform the computation normally
    if(lBound < uBound)
        return position <= uBound && position >=lBound;
    // Otherwise, check if the value is outside either bound and negate the response
    return position <= uBound || position >= lBound;
}

/// Current tolerance is &pm;0.1 degree w.r.t. the current number of counts per rotation
auto ArmMotor::hasReachedTarget(uint32_t targetCounts) const -> bool{
    auto tol = ArmMotor::TOLERANCE_RATIO * static_cast<double>(std::abs(this->COUNTS_PER_ROTATION));
    return ArmMotor::hasReachedTarget(targetCounts, std::max(1.0,tol));
}

auto ArmMotor::getMotorState() const -> MotorState{
    return this->currState;
}

void ArmMotor::setPower(float power){
    this->setPower(power, power == 0.F ? MotorState::STOP : MotorState::MOVING);
}

/// This method auto-publishes the speed command to the WRoboclaws
void ArmMotor::setPower(float power, MotorState state){
    // Check the bounds of the parameter
    if(abs(power) > 1) throw std::invalid_argument{std::string{"Power "} + std::to_string(power) + " is not on the interval [-1, 1]"};

    // Set up and send the speed message
    this->power = power;
    this->powerMsg.data = power * INT16_MAX;
    this->speedPub.publish(this->powerMsg);
    // Update the cur.nt motor state based on the power command input
    this->currState = state;
}

void ArmMotor::runToTarget(uint32_t targetCounts, float power, bool block){
    this->target = targetCounts; 
    this->maxPower = abs(power);
    // If we are not at our target...
    if(!this->hasReachedTarget(targetCounts)){
        // std::cout << "has not reached target" << std::endl;
        // Set the power in the correct direction and continue running to the target
        this->currState = MotorState::RUN_TO_TARGET;
        std_msgs::Float64 setpointMsg;
        setpointMsg.data = ArmMotor::encToRad(targetCounts);
        this->targetPub.publish(setpointMsg);

        // long int direction = targetCounts - this->getEncoderCounts();
        // power = abs(power) * (corrMod(direction, ((long int)this->COUNTS_PER_ROTATION)) < corrMod(-direction, ((long int)this->COUNTS_PER_ROTATION)) ? 1 : -1);
        // this->setPower(power, MotorState::RUN_TO_TARGET);

    // Otherwise, stop the motor
    } else {
        // std::cout << "has reached target" << std::endl;
        this->setPower(0.F, MotorState::STOP);
    }
    // If this is a blocking call...
    if(block){
        // Wait until the motor has reached the target, then stop
        while(!this->hasReachedTarget(targetCounts));
        this->setPower(0.F, MotorState::RUN_TO_TARGET);
    }
}

void ArmMotor::runToTarget(double rads, float power){
    // std::cout << "run to target: " << rads << ":" << this->radToEnc(rads) << ":" << this->getEncoderCounts() << std::endl;
    
    runToTarget(this->radToEnc(rads), power, false);
}

auto ArmMotor::getMotorName() const -> std::string{
    return this->motorName;
}

auto ArmMotor::getMotorID() const -> unsigned int{
    return this->motorID;
}

auto ArmMotor::getControllerID() const -> unsigned int{
    return this->controllerID;
}

auto ArmMotor::getPower() const -> float{
    return this->power;
}