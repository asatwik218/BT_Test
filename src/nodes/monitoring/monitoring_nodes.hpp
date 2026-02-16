#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "nodes/motor/motor_nodes.hpp"
#include "nodes/flexiv/flexiv_nodes.hpp"
#include "nodes/amr/amr_nodes_new.hpp"

#include <iostream>

namespace monitoring_bt {

// ============================================================================
// CheckMotorFaults — polls all connected motors for error codes
// Returns SUCCESS if no faults, FAILURE if any motor has an error
// ============================================================================
class CheckMotorFaults : public BT::SyncActionNode {
public:
    CheckMotorFaults(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("num_motors", 2, "Number of motors to check"),
            BT::OutputPort<int>("fault_motor_index", "Index of first faulted motor (-1 if none)"),
            BT::OutputPort<int>("fault_error_code", "Error code of faulted motor")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// CheckRobotFaults — checks Flexiv robot fault state
// Returns SUCCESS if robot is healthy, FAILURE if faulted
// ============================================================================
class CheckRobotFaults : public BT::SyncActionNode {
public:
    CheckRobotFaults(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("serial", "Robot serial number")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// CheckAMRFaults — checks AMR blocked state and battery level
// Returns SUCCESS if OK, FAILURE if blocked or battery critical
// ============================================================================
class CheckAMRFaults : public BT::SyncActionNode {
public:
    CheckAMRFaults(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<float>("min_battery_level", 10.0f, "Minimum battery level (%)"),
            BT::OutputPort<bool>("is_blocked", "Whether AMR is blocked"),
            BT::OutputPort<float>("battery_level", "Current battery level")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Registration
// ============================================================================
inline void registerMonitoringNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<CheckMotorFaults>("CheckMotorFaults");
    factory.registerNodeType<CheckRobotFaults>("CheckRobotFaults");
    factory.registerNodeType<CheckAMRFaults>("CheckAMRFaults");
}

} // namespace monitoring_bt
