#pragma once

#include <atomic>
#include <thread>
#include <iostream>
#include <conio.h>

namespace core {

class EmergencyStopManager {
public:
    static EmergencyStopManager& instance() {
        static EmergencyStopManager inst;
        return inst;
    }

    void start() {
        if (running_) return;
        running_ = true;
        halt_requested_ = false;

        monitor_thread_ = std::thread([this]() {
            std::cout << "[EmergencyStop] Press ESC to halt all operations." << std::endl;
            while (running_) {
                if (_kbhit()) {
                    int key = _getch();
                    if (key == 27) {
                        std::cout << "\n[EmergencyStop] *** ESC PRESSED - HALTING ALL OPERATIONS ***" << std::endl;
                        halt_requested_ = true;
                        return;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
        });
    }

    void stop() {
        running_ = false;
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
    }

    bool isHaltRequested() const { return halt_requested_.load(); }

    void reset() { halt_requested_ = false; }

private:
    EmergencyStopManager() = default;
    ~EmergencyStopManager() { stop(); }
    EmergencyStopManager(const EmergencyStopManager&) = delete;
    EmergencyStopManager& operator=(const EmergencyStopManager&) = delete;

    std::atomic<bool> halt_requested_{false};
    std::atomic<bool> running_{false};
    std::thread monitor_thread_;
};

} // namespace core
