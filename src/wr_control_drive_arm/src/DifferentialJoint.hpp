#include "AbstractJoint.hpp"

class DifferentialJoint : public AbstractJoint {
    public:
        DifferentialJoint(std::unique_ptr<ArmMotor> leftMotor, std::unique_ptr<ArmMotor> rightMotor, ros::NodeHandle &n,
                          const std::string &pitchTopicName, const std::string &rollTopicName,
                          const std::string &leftTopicName, const std::string &rightTopicName);

        auto getMotorPositions(const vector<double> &jointPositions) -> vector<double> override;
        auto getMotorVelocities(const vector<double> &jointVelocities) -> vector<double> override;
        auto getJointPositions(const vector<double> &motorPositions) -> vector<double> override;

    private:
        static constexpr uint32_t DEGREES_OF_FREEDOM = 2;
        static constexpr uint32_t MESSAGE_CACHE_SIZE = 10;
        // linear transformations works for position and velocity
        // [0.5 0.5]   [left motor ]    [pitch]
        // [ -1  1 ] * [right motor]  = [roll ]
        // double motorToJointMatrix[2][2] = {{0.5, 0.5}, {-1, 1}};
        static constexpr std::array<std::array<double, 2>, 2> MOTOR_TO_JOINT_MATRIX{{{1, 0}, {0, 1}}};
        // [1 -0.5]   [pitch]    [left motor ]
        // [1  0.5] * [roll ]  = [right motor]
        // double jointToMotorMatrix[2][2] = {{1, 0.5}, {1, -0.5}};
        static constexpr std::array<std::array<double, 2>, 2> JOINT_TO_MOTOR_MATRIX{{{1, 0}, {0, 1}}};

        void handoffPitchOutput(const std_msgs::Float64::ConstPtr&);
        void handoffRollOutput(const std_msgs::Float64::ConstPtr&);
        void handOffAllOutput();
        ros::Subscriber pitchOutputSubscriber;
        ros::Subscriber rollOutputSubscriber;
        ros::Publisher leftOutputPublisher;
        ros::Publisher rightOutputPublisher;
        float cachedPitchOutput = 0.0;
        float cachedRollOutput = 0.0;
        bool hasNewPitchOutput = false;
        bool hasNewRollOutput = false;

        void handoffLeftFeedback(const std_msgs::Float64::ConstPtr&);
        void handoffRightFeedback(const std_msgs::Float64::ConstPtr&);
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