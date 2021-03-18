class ArmMotor{
    private:
        enum MotorState{
            STOP, MOVING, RUN_TO_TARGET
        };
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
        MotorState getMotorState();
        void setPower(float power);
};