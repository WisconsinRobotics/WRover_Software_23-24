#include "AbstractJoint.hpp"
using std::vector;

class SimpleJoint : public AbstractJoint {
    public:
        ~SimpleJoint();
        SimpleJoint(ArmMotor* motor, ros::NodeHandle* n);

        int a(){return 2;}
        int b(){return 3;}

        void getMotorPositions(vector<double> &jointPositions, vector<double> &target);
        void getMotorVelocities(vector<double> &ointVelocities, vector<double> &target);
        void getJointPositions(vector<double> &motorPositions, vector<double> &target);

        void configMotor(ArmMotor* motor);
        void configVelocityHandshake(std::string jointTopicName, std::string motorTopicName);
        void handoffOutput(std_msgs::Float64);
        void handoffFeedback(std_msgs::Float64);

    private:
        ros::Subscriber jointOutputSubscriber;
        ros::Publisher motorOutputPublisher;

        ros::Subscriber motorFeedbackSubscriber;
        ros::Publisher jointFeedbackPublisher;
        
};
