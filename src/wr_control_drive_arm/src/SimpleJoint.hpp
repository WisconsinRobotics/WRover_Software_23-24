/**
 * @file SimpleJoint.hpp
 * @author Nichols Underwood
 * @brief Header file of the SimpleJoint class
 * @date 2021-10-25
 */

#include "AbstractJoint.hpp"

using std::vector;

class SimpleJoint : public AbstractJoint {
    public:
        SimpleJoint(std::unique_ptr<ArmMotor> motor, ros::NodeHandle& n);
        auto getMotorPositions(const vector<double> &jointPositions) -> vector<double> override;
        auto getMotorVelocities(const vector<double> &joinVelocities) -> vector<double> override;
        auto getJointPositions(const vector<double> &motorPositions) -> vector<double> override;
};
