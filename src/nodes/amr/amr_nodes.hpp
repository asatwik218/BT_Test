#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "Cy_AMR_Interface.h"

#include <string>
#include <iostream>

namespace amr_bt {

// ============================================================================
// Node 1: ConnectAMR
// ============================================================================
/**
 * @brief Connects to an AMR at the given IP
 *
 * Ports:
 *   - ip [input]: Robot IP address
 *   - amr_type [input]: AMR type string ("SEER"), default "SEER"
 */
class ConnectAMR : public BT::SyncActionNode {
public:
    ConnectAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type: SEER")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 2: DisconnectAMR
// ============================================================================
class DisconnectAMR : public BT::SyncActionNode {
public:
    DisconnectAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type: SEER")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 3: RelocateAMR
// ============================================================================
/**
 * @brief Sends a relocation (re-localization) command to the AMR
 *
 * Ports:
 *   - ip [input]: AMR IP address
 *   - amr_type [input]: AMR type (default "SEER")
 *   - auto_reloc [input]: Enable automatic relocation (default true)
 *   - x [input]: X coordinate in world frame (m), default 0.0
 *   - y [input]: Y coordinate in world frame (m), default 0.0
 *   - angle [input]: Orientation angle (rad), default 0.0
 *   - area_radius [input]: Search area radius (m), default 0.0
 */
class RelocateAMR : public BT::SyncActionNode {
public:
    RelocateAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<bool>("auto_reloc", true, "Enable automatic relocation"),
            BT::InputPort<double>("x", 0.0, "X coordinate (m)"),
            BT::InputPort<double>("y", 0.0, "Y coordinate (m)"),
            BT::InputPort<double>("angle", 0.0, "Orientation angle (rad)"),
            BT::InputPort<double>("area_radius", 0.0, "Search area radius (m)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 4: TranslateAMR
// ============================================================================
/**
 * @brief Sends a linear translation command
 */
class TranslateAMR : public BT::SyncActionNode {
public:
    TranslateAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<float>("distance", "Translation distance (m)"),
            BT::InputPort<float>("vel_x", 0.0f, "Velocity along X (m/s)"),
            BT::InputPort<float>("vel_y", 0.0f, "Velocity along Y (m/s)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 5: RotateAMR
// ============================================================================
/**
 * @brief Sends an angular rotation command
 */
class RotateAMR : public BT::SyncActionNode {
public:
    RotateAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<float>("angle", "Rotation angle (rad)"),
            BT::InputPort<float>("ang_vel", 0.0f, "Angular velocity (rad/s)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 6: GoToAMR
// ============================================================================
/**
 * @brief Commands the AMR to navigate to a target location
 */
class GoToAMR : public BT::SyncActionNode {
public:
    GoToAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<std::string>("source_id", "SELF_POSITION", "Source position ID"),
            BT::InputPort<std::string>("dest_id", "Destination position ID"),
            BT::InputPort<std::string>("movement_mode", "forward", "Movement mode"),
            BT::InputPort<float>("arrival_angle", 0.0f, "Desired arrival orientation (rad)"),
            BT::InputPort<float>("max_vel", 0.0f, "Max linear velocity (m/s)"),
            BT::InputPort<float>("max_ang_vel", 0.0f, "Max angular velocity (rad/s)"),
            BT::InputPort<float>("max_acc", 0.0f, "Max linear acceleration (m/s^2)"),
            BT::InputPort<float>("max_ang_acc", 0.0f, "Max angular acceleration (rad/s^2)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 7: PauseAMR
// ============================================================================
class PauseAMR : public BT::SyncActionNode {
public:
    PauseAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 8: ResumeAMR
// ============================================================================
class ResumeAMR : public BT::SyncActionNode {
public:
    ResumeAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 9: CancelAMR
// ============================================================================
class CancelAMR : public BT::SyncActionNode {
public:
    CancelAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 10: StopAMR
// ============================================================================
class StopAMR : public BT::SyncActionNode {
public:
    StopAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<bool>("emergency_stop", true, "Output emergency stop signal")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 11: GetAMRLocationStatus
// ============================================================================
/**
 * @brief Reads the AMR's current location
 */
class GetAMRLocationStatus : public BT::SyncActionNode {
public:
    GetAMRLocationStatus(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::OutputPort<float>("x", "X position (m)"),
            BT::OutputPort<float>("y", "Y position (m)"),
            BT::OutputPort<float>("angle", "Orientation angle (rad)"),
            BT::OutputPort<float>("confidence", "Localization confidence [0-1]")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 12: GetAMRNavigationStatus
// ============================================================================
/**
 * @brief Reads the AMR's navigation task status
 */
class GetAMRNavigationStatus : public BT::SyncActionNode {
public:
    GetAMRNavigationStatus(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::OutputPort<int>("task_status", "Task status: 0=NONE,1=WAITING,2=RUNNING,3=SUSPENDED,4=COMPLETED,5=FAILED,6=CANCELED"),
            BT::OutputPort<int>("task_type", "Task type")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 13: GetAMRBatteryStatus
// ============================================================================
class GetAMRBatteryStatus : public BT::SyncActionNode {
public:
    GetAMRBatteryStatus(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::OutputPort<float>("battery_level", "Battery level [0-1]"),
            BT::OutputPort<bool>("charging", "Whether battery is charging"),
            BT::OutputPort<float>("voltage", "Battery voltage (V)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 14: GetAMRLocalizationStatus
// ============================================================================
class GetAMRLocalizationStatus : public BT::SyncActionNode {
public:
    GetAMRLocalizationStatus(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::OutputPort<int>("reloc_status", "0=INIT, 1=SUCCESS, 2=RELOCING")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 15: StartContinuousTelemetryAMR
// ============================================================================
class StartContinuousTelemetryAMR : public BT::SyncActionNode {
public:
    StartContinuousTelemetryAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<int>("interval_ms", 100, "Telemetry update interval (ms)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 16: ReadContinuousTelemetryAMR
// ============================================================================
/**
 * @brief Reads the latest continuous telemetry snapshot
 */
class ReadContinuousTelemetryAMR : public BT::SyncActionNode {
public:
    ReadContinuousTelemetryAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::OutputPort<float>("x", "X position (m)"),
            BT::OutputPort<float>("y", "Y position (m)"),
            BT::OutputPort<float>("angle", "Orientation angle (rad)"),
            BT::OutputPort<float>("vel_x", "Linear velocity X (m/s)"),
            BT::OutputPort<float>("vel_y", "Linear velocity Y (m/s)"),
            BT::OutputPort<float>("battery_level", "Battery level [0-1]"),
            BT::OutputPort<int>("task_status", "Navigation task status"),
            BT::OutputPort<bool>("blocked", "Whether robot is blocked")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 17: StopContinuousTelemetryAMR
// ============================================================================
class StopContinuousTelemetryAMR : public BT::SyncActionNode {
public:
    StopContinuousTelemetryAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 18: LiftAMR
// ============================================================================
class LiftAMR : public BT::SyncActionNode {
public:
    LiftAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
            BT::InputPort<float>("height", "Target lift height (m)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 19: StopLiftAMR
// ============================================================================
class StopLiftAMR : public BT::SyncActionNode {
public:
    StopLiftAMR(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("ip", "AMR IP address"),
            BT::InputPort<std::string>("amr_type", "SEER", "AMR type")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Helper: parse AMRType from string
// ============================================================================
inline AMRType parseAMRType(const std::string& type_str) {
    if (type_str == "SEER") return AMRType::SEER;
    return AMRType::UNKNOWN;
}

// ============================================================================
// Registration helper
// ============================================================================
inline void registerAMRNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ConnectAMR>("ConnectAMR");
    factory.registerNodeType<DisconnectAMR>("DisconnectAMR");
    factory.registerNodeType<RelocateAMR>("RelocateAMR");
    factory.registerNodeType<TranslateAMR>("TranslateAMR");
    factory.registerNodeType<RotateAMR>("RotateAMR");
    factory.registerNodeType<GoToAMR>("GoToAMR");
    factory.registerNodeType<PauseAMR>("PauseAMR");
    factory.registerNodeType<ResumeAMR>("ResumeAMR");
    factory.registerNodeType<CancelAMR>("CancelAMR");
    factory.registerNodeType<StopAMR>("StopAMR");
    factory.registerNodeType<GetAMRLocationStatus>("GetAMRLocationStatus");
    factory.registerNodeType<GetAMRNavigationStatus>("GetAMRNavigationStatus");
    factory.registerNodeType<GetAMRBatteryStatus>("GetAMRBatteryStatus");
    factory.registerNodeType<GetAMRLocalizationStatus>("GetAMRLocalizationStatus");
    factory.registerNodeType<StartContinuousTelemetryAMR>("StartContinuousTelemetryAMR");
    factory.registerNodeType<ReadContinuousTelemetryAMR>("ReadContinuousTelemetryAMR");
    factory.registerNodeType<StopContinuousTelemetryAMR>("StopContinuousTelemetryAMR");
    factory.registerNodeType<LiftAMR>("LiftAMR");
    factory.registerNodeType<StopLiftAMR>("StopLiftAMR");
}

} // namespace amr_bt
