#include "gripper_nodes.hpp"

#include <iostream>
#include <thread>
#include <chrono>

namespace gripper_bt {

// ============================================================================
// ConnectGripper Implementation
// ============================================================================

BT::NodeStatus ConnectGripper::tick() {
    auto port = getInput<std::string>("port");
    auto gripper_type = getInput<std::string>("gripper_type");

    if (!port) {
        std::cerr << "[ConnectGripper] Missing required input 'port': " << port.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!gripper_type) {
        std::cerr << "[ConnectGripper] Missing required input 'gripper_type': " << gripper_type.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        std::cout << "[ConnectGripper] Connecting to " << gripper_type.value()
                  << " gripper on port: " << port.value() << std::endl;

        auto gripper = GripperManager::instance().create(port.value(), gripper_type.value());
        if (!gripper) {
            std::cerr << "[ConnectGripper] Failed to create gripper" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Initialize the gripper
        std::cout << "[ConnectGripper] Initializing gripper..." << std::endl;
        auto init_result = gripper->initialize();
        if (!init_result.has_value()) {
            std::cerr << "[ConnectGripper] Initialization failed: "
                      << init_result.error().message << std::endl;
            GripperManager::instance().remove(port.value());
            return BT::NodeStatus::FAILURE;
        }

        // Wait for initialization to complete
        const int max_wait_ms = 10000;
        const int poll_interval_ms = 100;
        int waited_ms = 0;

        while (waited_ms < max_wait_ms) {
            auto status = gripper->getStatus();
            if (status.has_value() && status->is_initialized) {
                std::cout << "[ConnectGripper] Gripper initialized successfully on port: "
                          << port.value() << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_ms));
            waited_ms += poll_interval_ms;
        }

        std::cerr << "[ConnectGripper] Gripper initialization timeout" << std::endl;
        GripperManager::instance().remove(port.value());
        return BT::NodeStatus::FAILURE;

    } catch (const std::exception& e) {
        std::cerr << "[ConnectGripper] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================================
// DisconnectGripper Implementation
// ============================================================================

BT::NodeStatus DisconnectGripper::tick() {
    auto port = getInput<std::string>("port");
    if (!port) {
        std::cerr << "[DisconnectGripper] Missing 'port': " << port.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[DisconnectGripper] Disconnecting gripper on port: " << port.value() << std::endl;

    // Remove from manager - destructor will handle cleanup
    GripperManager::instance().remove(port.value());

    std::cout << "[DisconnectGripper] Gripper disconnected" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// OpenGripperAsync Implementation
// ============================================================================

BT::NodeStatus OpenGripperAsync::onStart() {
    auto port = getInput<std::string>("port");
    if (!port) {
        std::cerr << "[OpenGripperAsync] Missing 'port'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    port_ = port.value();
    timeout_ms_ = getInput<int>("timeout_ms").value_or(5000);
    elapsed_ms_ = 0;

    try {
        auto gripper = GripperManager::instance().get(port_);
        if (!gripper) {
            std::cerr << "[OpenGripperAsync] Gripper not found on port: " << port_ << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        auto result = gripper->open();
        if (!result.has_value()) {
            std::cerr << "[OpenGripperAsync] Failed to open: " << result.error().message << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[OpenGripperAsync] Opening gripper, waiting for completion..." << std::endl;
        return BT::NodeStatus::RUNNING;

    } catch (const std::exception& e) {
        std::cerr << "[OpenGripperAsync] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus OpenGripperAsync::onRunning() {
    try {
        auto gripper = GripperManager::instance().get(port_);
        if (!gripper) return BT::NodeStatus::FAILURE;

        auto status = gripper->getStatus();
        if (status.has_value()) {
            if (!status->is_moving) {
                std::cout << "[OpenGripperAsync] Gripper opened successfully" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }

        elapsed_ms_ += 10;
        if (elapsed_ms_ >= timeout_ms_) {
            std::cerr << "[OpenGripperAsync] Timeout waiting for gripper to open" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;

    } catch (const std::exception& e) {
        std::cerr << "[OpenGripperAsync] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void OpenGripperAsync::onHalted() {
    std::cout << "[OpenGripperAsync] HALTED" << std::endl;
}

// ============================================================================
// CloseGripperAsync Implementation
// ============================================================================

BT::NodeStatus CloseGripperAsync::onStart() {
    auto port = getInput<std::string>("port");
    if (!port) {
        std::cerr << "[CloseGripperAsync] Missing 'port'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    port_ = port.value();
    timeout_ms_ = getInput<int>("timeout_ms").value_or(5000);
    elapsed_ms_ = 0;

    try {
        auto gripper = GripperManager::instance().get(port_);
        if (!gripper) {
            std::cerr << "[CloseGripperAsync] Gripper not found on port: " << port_ << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        auto result = gripper->close();
        if (!result.has_value()) {
            std::cerr << "[CloseGripperAsync] Failed to close: " << result.error().message << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[CloseGripperAsync] Closing gripper, waiting for completion..." << std::endl;
        return BT::NodeStatus::RUNNING;

    } catch (const std::exception& e) {
        std::cerr << "[CloseGripperAsync] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus CloseGripperAsync::onRunning() {
    try {
        auto gripper = GripperManager::instance().get(port_);
        if (!gripper) return BT::NodeStatus::FAILURE;

        auto status = gripper->getStatus();
        if (status.has_value()) {
            if (!status->is_moving) {
                std::cout << "[CloseGripperAsync] Gripper closed successfully" << std::endl;
                return BT::NodeStatus::SUCCESS;
            }
        }

        elapsed_ms_ += 10;
        if (elapsed_ms_ >= timeout_ms_) {
            std::cerr << "[CloseGripperAsync] Timeout waiting for gripper to close" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;

    } catch (const std::exception& e) {
        std::cerr << "[CloseGripperAsync] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void CloseGripperAsync::onHalted() {
    std::cout << "[CloseGripperAsync] HALTED" << std::endl;
}

} // namespace gripper_bt
