#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "flexiv/rdk/robot.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <array>
#include <cmath>
#include <string>
#include <sstream>
#include <iostream>

// Template specialization for BT::convertFromString to handle std::array<double, 3>
// This must be in the BT namespace before any usage
namespace BT {

template <>
inline std::array<double, 3> convertFromString(StringView str) {
    std::array<double, 3> result = {0.0, 0.0, 0.0};

    // Parse semicolon-separated values: "x;y;z"
    std::string input(str.data(), str.size());
    std::istringstream iss(input);
    std::string token;
    size_t index = 0;

    while (std::getline(iss, token, ';') && index < 3) {
        result[index++] = std::stod(token);
    }

    if (index != 3) {
        throw RuntimeError("Cannot convert '", str, "' to std::array<double, 3>. Expected format: 'x;y;z'");
    }

    return result;
}

} // namespace BT

namespace flexiv_bt {

/**
 * @brief Singleton manager for Flexiv robot instances.
 * Stores robot objects by serial number for shared access across BT nodes.
 */
class RobotManager {
public:
    static RobotManager& instance() {
        static RobotManager instance;
        return instance;
    }

    /**
     * @brief Get existing robot or create new connection
     * @param serial Robot serial number (e.g., "Rizon4s-123456")
     * @return Shared pointer to robot instance
     */
    std::shared_ptr<flexiv::rdk::Robot> getOrCreate(const std::string& serial) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = robots_.find(serial);
        if (it != robots_.end()) {
            std::cout << "[RobotManager] Returning existing robot: " << serial << std::endl;
            return it->second;
        }
        std::cout << "[RobotManager] Creating new robot connection: " << serial << std::endl;
        auto robot = std::make_shared<flexiv::rdk::Robot>(serial);
        std::cout << "[RobotManager] Robot created, use_count: " << robot.use_count() << std::endl;
        robots_[serial] = robot;
        std::cout << "[RobotManager] Robot stored in map, use_count: " << robot.use_count() << std::endl;
        return robot;
    }

    /**
     * @brief Get existing robot connection
     * @param serial Robot serial number
     * @return Shared pointer to robot, or nullptr if not found
     */
    std::shared_ptr<flexiv::rdk::Robot> get(const std::string& serial) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = robots_.find(serial);
        if (it != robots_.end()) {
            return it->second;
        }
        return nullptr;
    }

    /**
     * @brief Remove robot connection (destructor will disconnect)
     * @param serial Robot serial number
     */
    void remove(const std::string& serial) {
        std::lock_guard<std::mutex> lock(mutex_);
        robots_.erase(serial);
    }

    /**
     * @brief Check if robot exists in manager
     * @param serial Robot serial number
     * @return true if robot exists
     */
    bool exists(const std::string& serial) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return robots_.find(serial) != robots_.end();
    }

private:
    RobotManager() = default;
    ~RobotManager() = default;
    RobotManager(const RobotManager&) = delete;
    RobotManager& operator=(const RobotManager&) = delete;

    std::map<std::string, std::shared_ptr<flexiv::rdk::Robot>> robots_;
    mutable std::mutex mutex_;
};

// ============================================================================
// Node 1: ConnectRobot
// ============================================================================
/**
 * @brief Connects to a Flexiv robot and stores connection in RobotManager
 *
 * Ports:
 *   - serial [input]: Robot serial number (e.g., "Rizon4s-123456")
 *
 * Returns SUCCESS if connected, FAILURE on error
 */
class ConnectRobot : public BT::SyncActionNode {
public:
    ConnectRobot(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("serial", "Robot serial number")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 2: StopRobot
// ============================================================================
/**
 * @brief Stops robot motion and transitions to IDLE mode
 *
 * Ports:
 *   - serial [input]: Robot serial number
 *
 * Returns SUCCESS if stopped, FAILURE on error
 */
class StopRobot : public BT::SyncActionNode {
public:
    StopRobot(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("serial", "Robot serial number")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 5: DisconnectRobot
// ============================================================================
/**
 * @brief Disconnects robot and removes from RobotManager
 *
 * Ports:
 *   - serial [input]: Robot serial number
 *
 * Returns SUCCESS after disconnection
 */
class DisconnectRobot : public BT::SyncActionNode {
public:
    DisconnectRobot(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("serial", "Robot serial number")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// MoveRobotAsync — sends motion in onStart, polls target in onRunning
// ============================================================================
class MoveRobotAsync : public BT::StatefulActionNode {
public:
    MoveRobotAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("serial", "Robot serial number"),
            BT::InputPort<std::array<double, 3>>("position_mm", "Target position [x,y,z] in mm"),
            BT::InputPort<std::array<double, 3>>("orientation_deg", "Target orientation [roll,pitch,yaw] in degrees (ZYX)"),
            BT::InputPort<double>("max_linear_vel", 0.5, "Max linear velocity (m/s)"),
            BT::InputPort<double>("max_angular_vel", 1.0, "Max angular velocity (rad/s)"),
            BT::InputPort<double>("max_linear_acc", 2.0, "Max linear acceleration (m/s^2)"),
            BT::InputPort<double>("max_angular_acc", 5.0, "Max angular acceleration (rad/s^2)"),
            BT::InputPort<double>("position_tolerance_mm", 2.0, "Position tolerance (mm)"),
            BT::InputPort<double>("orientation_tolerance_deg", 2.0, "Orientation tolerance (degrees)")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string serial_;
    std::array<double, 3> target_pos_mm_{};
    std::array<double, 4> target_quat_{};
    double pos_tol_mm_ = 2.0;
    double orient_tol_deg_ = 2.0;
};

// ============================================================================
// Registration helper
// ============================================================================
inline void registerFlexivNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ConnectRobot>("ConnectRobot");
    factory.registerNodeType<StopRobot>("StopRobot");
    factory.registerNodeType<DisconnectRobot>("DisconnectRobot");
    factory.registerNodeType<MoveRobotAsync>("MoveRobotAsync");
}

} // namespace flexiv_bt
