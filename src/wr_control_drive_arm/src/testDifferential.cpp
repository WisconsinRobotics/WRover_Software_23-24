#include <cstdint>
#include "Motor.hpp"
#include "DifferentialJointToMotorSpeedConverter.hpp"
#include "ros/init.h"
#include <ros/ros.h>

auto main(int32_t argc, char** argv) -> int32_t{
    ros::init(argc, argv, "testNode");
    
    ros::NodeHandle n{};
    using std::literals::string_literals::operator""s;

    const auto wristLeftMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::A, n)};
    const auto wristRightMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::B, n)};

    const auto differentialSpeedConverter{std::make_shared<DifferentialJointToMotorSpeedConverter>(wristLeftMotor, wristRightMotor)};

    ros::Rate loopRate{50};
    while(ros::ok()){
        differentialSpeedConverter->setPitchSpeed(0.3);
        differentialSpeedConverter->setRollSpeed(0);
        loopRate.sleep();
    }

    return 0;
}