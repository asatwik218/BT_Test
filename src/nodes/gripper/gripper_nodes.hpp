#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "cynlr_gripper/IGripper.hpp"
#include "cynlr_gripper/DHAG/DHAGGripper.hpp"
#include "cynlr_gripper/Robotiq2F/Robotiq2FGripper.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <iostream>

namespace gripper_bt {

/**
 * @brief Singleton manager for gripper instances.
 * Stores gripper objects by port name for shared access across BT nodes.
 */
class GripperManager {
public:
    static GripperManager& instance() {
        static GripperManager instance;
        return instance;
    }

    /**
     * @brief Create a new gripper connection
     * @param port Serial port (e.g., "COM3" or "/dev/ttyUSB0")
     * @param gripper_type Type of gripper: "DHAG" or "Robotiq2F"
     * @return Shared pointer to gripper instance, or nullptr on failure
     */
    std::shared_ptr<cynlr::gripper::IGripper> create(
        const std::string& port,
        const std::string& gripper_type
    ) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Check if already exists
        auto it = grippers_.find(port);
        if (it != grippers_.end()) {
            std::cout << "[GripperManager] Gripper already exists on port: " << port << std::endl;
            return it->second;
        }

        std::shared_ptr<cynlr::gripper::IGripper> gripper;

        try {
            if (gripper_type == "DHAG") {
                std::cout << "[GripperManager] Creating DHAGGripper on port: " << port << std::endl;
                gripper = std::make_shared<cynlr::gripper::DHAGGripper>(port);
            } else if (gripper_type == "Robotiq2F") {
                std::cout << "[GripperManager] Creating Robotiq2FGripper on port: " << port << std::endl;
                gripper = std::make_shared<cynlr::gripper::Robotiq2FGripper>(port);
            } else {
                std::cerr << "[GripperManager] Unknown gripper type: " << gripper_type << std::endl;
                return nullptr;
            }

            grippers_[port] = gripper;
            std::cout << "[GripperManager] Gripper created successfully" << std::endl;
            return gripper;

        } catch (const std::exception& e) {
            std::cerr << "[GripperManager] Failed to create gripper: " << e.what() << std::endl;
            return nullptr;
        }
    }

    /**
     * @brief Get existing gripper connection
     * @param port Serial port
     * @return Shared pointer to gripper, or nullptr if not found
     */
    std::shared_ptr<cynlr::gripper::IGripper> get(const std::string& port) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = grippers_.find(port);
        if (it != grippers_.end()) {
            return it->second;
        }
        return nullptr;
    }

    /**
     * @brief Remove gripper connection
     * @param port Serial port
     */
    void remove(const std::string& port) {
        std::lock_guard<std::mutex> lock(mutex_);
        grippers_.erase(port);
        std::cout << "[GripperManager] Gripper removed from port: " << port << std::endl;
    }

    /**
     * @brief Check if gripper exists
     * @param port Serial port
     * @return true if gripper exists
     */
    bool exists(const std::string& port) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return grippers_.find(port) != grippers_.end();
    }

private:
    GripperManager() = default;
    ~GripperManager() = default;
    GripperManager(const GripperManager&) = delete;
    GripperManager& operator=(const GripperManager&) = delete;

    std::map<std::string, std::shared_ptr<cynlr::gripper::IGripper>> grippers_;
    mutable std::mutex mutex_;
};

// ============================================================================
// Node 1: ConnectGripper
// ============================================================================
/**
 * @brief Connects to a gripper and initializes it
 *
 * Ports:
 *   - port [input]: Serial port (e.g., "COM3" or "/dev/ttyUSB0")
 *   - gripper_type [input]: Type of gripper ("DHAG" or "Robotiq2F")
 *
 * Returns SUCCESS if connected and initialized, FAILURE on error
 */
class ConnectGripper : public BT::SyncActionNode {
public:
    ConnectGripper(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port", "Serial port (e.g., COM3 or /dev/ttyUSB0)"),
            BT::InputPort<std::string>("gripper_type", "Gripper type: DHAG or Robotiq2F")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 2: DisconnectGripper
// ============================================================================
/**
 * @brief Disconnects gripper and removes from GripperManager
 *
 * Ports:
 *   - port [input]: Serial port
 *
 * Returns SUCCESS after disconnection
 */
class DisconnectGripper : public BT::SyncActionNode {
public:
    DisconnectGripper(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port", "Serial port")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// OpenGripperAsync — sends open in onStart, polls gripper state in onRunning
// ============================================================================
class OpenGripperAsync : public BT::StatefulActionNode {
public:
    OpenGripperAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port", "Serial port (e.g., COM3)"),
            BT::InputPort<int>("timeout_ms", 5000, "Timeout in milliseconds")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string port_;
    int timeout_ms_ = 5000;
    int elapsed_ms_ = 0;
};

// ============================================================================
// CloseGripperAsync — sends close in onStart, polls gripper state in onRunning
// ============================================================================
class CloseGripperAsync : public BT::StatefulActionNode {
public:
    CloseGripperAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("port", "Serial port (e.g., COM3)"),
            BT::InputPort<int>("timeout_ms", 5000, "Timeout in milliseconds")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string port_;
    int timeout_ms_ = 5000;
    int elapsed_ms_ = 0;
};

// ============================================================================
// Registration helper
// ============================================================================
inline void registerGripperNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ConnectGripper>("ConnectGripper");
    factory.registerNodeType<DisconnectGripper>("DisconnectGripper");
    factory.registerNodeType<OpenGripperAsync>("OpenGripperAsync");
    factory.registerNodeType<CloseGripperAsync>("CloseGripperAsync");
}

} // namespace gripper_bt
