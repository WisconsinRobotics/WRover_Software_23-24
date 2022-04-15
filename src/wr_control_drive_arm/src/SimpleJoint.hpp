#include "AbstractJoint.hpp"

using std::vector;

class SimpleJoint : public AbstractJoint {
    public:
        SimpleJoint(std::unique_ptr<ArmMotor> motor, ros::NodeHandle& n);
        void getMotorPositions(const vector<double> &jointPositions, vector<double> &target) override;
        void getMotorVelocities(const vector<double> &ointVelocities, vector<double> &target) override;
        void getJointPositions(const vector<double> &motorPositions, vector<double> &target) override;
};
