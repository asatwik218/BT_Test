#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "nodes/amr/amr_nodes_new.hpp"

#include <string>
#include <iostream>

namespace amr_bt {

// ============================================================================
// GoToAMRAsync — sends GoTo in onStart, polls navigation status in onRunning
// ============================================================================
class GoToAMRAsync : public BT::StatefulActionNode {
public:
    GoToAMRAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<std::string>("source_id", "SELF_POSITION", "Source position ID"),
            BT::InputPort<std::string>("dest_id", "Destination position ID"),
            BT::InputPort<std::string>("movement_mode", "forward", "Movement mode"),
            BT::InputPort<float>("arrival_angle", 0.0f, "Arrival angle (rad)"),
            BT::InputPort<float>("max_vel", 0.0f, "Max velocity (m/s)"),
            BT::InputPort<float>("max_ang_vel", 0.0f, "Max angular velocity (rad/s)"),
            BT::InputPort<float>("max_acc", 0.0f, "Max acceleration (m/s^2)"),
            BT::InputPort<float>("max_ang_acc", 0.0f, "Max angular acceleration (rad/s^2)")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string ip_;
    AMRType amr_type_ = AMRType::SEER;
};

// ============================================================================
// TranslateAMRAsync — sends translation, polls navigation status
// ============================================================================
class TranslateAMRAsync : public BT::StatefulActionNode {
public:
    TranslateAMRAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<float>("distance", "Translation distance (m)"),
            BT::InputPort<float>("vel_x", 0.0f, "Velocity X (m/s)"),
            BT::InputPort<float>("vel_y", 0.0f, "Velocity Y (m/s)")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string ip_;
    AMRType amr_type_ = AMRType::SEER;
};

// ============================================================================
// RotateAMRAsync — sends rotation, polls navigation status
// ============================================================================
class RotateAMRAsync : public BT::StatefulActionNode {
public:
    RotateAMRAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<float>("angle", "Rotation angle (rad)"),
            BT::InputPort<float>("ang_vel", 0.0f, "Angular velocity (rad/s)")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::string ip_;
    AMRType amr_type_ = AMRType::SEER;
};

// ============================================================================
// Registration
// ============================================================================
inline void registerAMRNodesAsync(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<GoToAMRAsync>("GoToAMRAsync");
    factory.registerNodeType<TranslateAMRAsync>("TranslateAMRAsync");
    factory.registerNodeType<RotateAMRAsync>("RotateAMRAsync");
}

} // namespace amr_bt
