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
// OpenGripper Implementation
// ============================================================================

BT::NodeStatus OpenGripper::tick() {
    auto port = getInput<std::string>("port");
    if (!port) {
        std::cerr << "[OpenGripper] Missing 'port': " << port.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        auto gripper = GripperManager::instance().get(port.value());
        if (!gripper) {
            std::cerr << "[OpenGripper] Gripper not found on port: " << port.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[OpenGripper] Opening gripper on port: " << port.value() << std::endl;

        auto result = gripper->open();
        if (!result.has_value()) {
            std::cerr << "[OpenGripper] Failed to open gripper: "
                      << result.error().message << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[OpenGripper] Open command sent successfully" << std::endl;
        return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "[OpenGripper] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================================
// CloseGripper Implementation
// ============================================================================

BT::NodeStatus CloseGripper::tick() {
    auto port = getInput<std::string>("port");
    if (!port) {
        std::cerr << "[CloseGripper] Missing 'port': " << port.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        auto gripper = GripperManager::instance().get(port.value());
        if (!gripper) {
            std::cerr << "[CloseGripper] Gripper not found on port: " << port.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[CloseGripper] Closing gripper on port: " << port.value() << std::endl;

        auto result = gripper->close();
        if (!result.has_value()) {
            std::cerr << "[CloseGripper] Failed to close gripper: "
                      << result.error().message << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[CloseGripper] Close command sent successfully" << std::endl;
        return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "[CloseGripper] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

} // namespace gripper_bt
