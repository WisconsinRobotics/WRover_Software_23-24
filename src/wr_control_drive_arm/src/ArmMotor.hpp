enum MotorState{
            STOP, MOVING, RUN_TO_TARGET
        };
class ArmMotor{
    private:
        MotorState currState;
        std::string motorName;
        unsigned int controllerID;
        unsigned int motorID;
    public:
        ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle* n);
        // ~ArmMotor();
        int getEncoderCounts();
        // void resetEncoder();
        void runToTarget(int targetCounts, float power);
        void runToTarget(int targetCounts, float power, bool block);
        void runToTarget(double rads, float power);
        MotorState getMotorState();
        void setPower(float power);
};