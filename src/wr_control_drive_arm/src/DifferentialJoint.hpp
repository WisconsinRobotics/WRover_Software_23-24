#include "AbstractJoint.hpp"

class DifferentialJoint : public AbstractJoint {
    public:
        ~DifferentialJoint();
        DifferentialJoint(ArmMotor *leftMotor, ArmMotor *rightMotor, ros::NodeHandle *n);

        vector<double> getMotorPositions(vector<double> jointPositions);
        vector<double> getMotorVelocities(vector<double> jointVelocities);
        vector<double> getJointPositions(vector<double> motorPositions);

        void configVelocityHandshake(std::string pitchTopicName, std::string rollTopicName, std::string leftTopicName, std::string rightTopicName);

    private:
        // linear transformations works for position and velocity
        // [0.5 0.5]   [left motor ]    [pitch]
        // [ -1  1 ] * [right motor]  = [roll ]
        double motorToJointMatrix[2][2] = {{0.5, 0.5}, {-1, 1}};
        // [1 -0.5]   [pitch]    [left motor ]
        // [1  0.5] * [roll ]  = [right motor]
        double jointToMotorMatrix[2][2] = {{1, 0.5}, {1, -0.5}};

        void handoffPitchOutput(std_msgs::Float64);
        void handoffRollOutput(std_msgs::Float64);
        void handOffAllOutput();
        ros::Subscriber pitchOutputSubscriber;
        ros::Subscriber rollOutputSubscriber;
        ros::Publisher leftOutputPublisher;
        ros::Publisher rightOutputPublisher;
        float cachedPitchOutput = 0.0;
        float cachedRollOutput = 0.0;
        bool hasNewPitchOutput = false;
        bool hasNewRollOutput = false;

        void handoffLeftFeedback(std_msgs::Float64);
        void handoffRightFeedback(std_msgs::Float64);
        void handOffAllFeedback();
        ros::Subscriber leftFeedbackSubscriber;
        ros::Subscriber rightFeedbackSubscriber;
        ros::Publisher pitchFeedbackPublisher;
        ros::Publisher rollFeedbackPublisher;
        float cachedLeftFeedback = 0.0;
        float cachedRightFeedback = 0.0;
        bool hasNewLeftFeedback = false;
        bool hasNewRightFeedback = false;
};