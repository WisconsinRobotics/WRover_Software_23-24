#include "DifferentialJointToMotorSpeedConverter.hpp"
#include "Motor.hpp"
#include "ros/init.h"
#include <cstdint>
#include <ros/ros.h>

auto main(int32_t argc, char **argv) -> int32_t {
    ros::init(argc, argv, "testNode");

    ros::NodeHandle nodeHandle{};
    using std::literals::string_literals::operator""s;

    const auto wristLeftMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::A, nodeHandle)};
    const auto wristRightMotor{std::make_shared<Motor>("aux2"s, RoboclawChannel::B, nodeHandle)};

    const auto differentialSpeedConverter{std::make_shared<DifferentialJointToMotorSpeedConverter>(wristLeftMotor, wristRightMotor)};

    constexpr auto SPEED_UPDATE_HZ{50};
    ros::Rate loopRate{SPEED_UPDATE_HZ};

    constexpr double PITCH_SPEED{0.3};
    constexpr double ROLL_SPEED{0};
    while (ros::ok()) {
        differentialSpeedConverter->setPitchSpeed(PITCH_SPEED);
        differentialSpeedConverter->setRollSpeed(ROLL_SPEED);
        loopRate.sleep();
    }

    return 0;
}