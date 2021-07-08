#include "Watchdog.hpp"
#include <chrono>

Watchdog::Watchdog(double timeout){
    this->timeout = timeout;
    this->pet();
}

void Watchdog::pet(){
    this->lastTime = std::chrono::system_clock::now();
}

bool Watchdog::isMad(){
    return std::chrono::duration<double>(std::chrono::system_clock::now() - this->lastTime).count() >= this->timeout;
}