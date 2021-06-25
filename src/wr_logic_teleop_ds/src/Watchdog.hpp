#include <ros/ros.h>
#include <chrono>

class Watchdog{
    private:
        std::chrono::time_point<std::chrono::system_clock> lastTime;
        double timeout;
    public:
        Watchdog(double timeout);
        void pet();
        bool isMad();
};
