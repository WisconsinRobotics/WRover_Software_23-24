#include "AbstractJoint.hpp"
using std::vector;

class SimpleJoint : public AbstractJoint {
    public:
        ~SimpleJoint();
        SimpleJoint(ArmMotor* motor, ros::NodeHandle* n);

        int a(){return 2;}
        int b(){return 3;}

        vector<double> getMotorPositions(vector<double> jointPositions);
        vector<double> getMotorVelocities(vector<double> jointVelocities);
        vector<double> getJointPositions(vector<double> motorPositions);

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
