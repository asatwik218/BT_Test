#pragma once

#include "behaviortree_cpp/bt_factory.h"

#include <Windows.h>
#include <cstdint>
#include <string>
#include <mutex>
#include <iostream>

namespace amr_bt {

// ============================================================================
// AMR type definitions (matching Cy_AMR_Interface.h structs exactly)
// ============================================================================

enum class AMRType { SEER = 0, UNKNOWN = 999 };

struct RelocParams {
    bool auto_reloc = true;
    double x = 0.0;
    double y = 0.0;
    double angle = 0.0;
    double area_radius = 0.0;
};

struct TranslationParams {
    float distance = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    int motion_mode = 0;
};

struct RotationParams {
    float angle = 0.0f;
    float ang_vel = 0.0f;
    int motion_mode = 0;
};

struct GoToParams {
    const char* source_id = "SELF_POSITION";
    const char* dest_id = "SELF_POSITION";
    const char* movement_mode = "forward";
    float arrival_angle = 0.0f;
    float max_vel = 0.0f;
    float max_ang_vel = 0.0f;
    float max_acc = 0.0f;
    float max_ang_acc = 0.0f;
};

struct StopParams {
    bool output_emergency_stop = true;
};

struct LocationData {
    float x = 0.0f;
    float y = 0.0f;
    float angle = 0.0f;
    float confidence = 0.0f;
    char* current_station = nullptr;
    char* last_station = nullptr;
    int localization_method = 0;
};

struct SpeedData {
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    float ang_vel = 0.0f;
};

struct BlockedData {
    bool blocked = false;
    int block_reason = -1;
    float block_x = 0.0f;
    float block_y = 0.0f;
};

struct BatteryData {
    float battery_level = 0.0f;
    float battery_temp = 0.0f;
    bool charging = false;
    float volatage = 0.0f;
    float current = 0.0f;
};

struct LocalizationData {
    int reloc_status = 0;
};

struct NavigationData {
    bool simple_data = false;
    int task_status = 0;
    int task_type = 0;
};

struct MetaData {
    char* upload_timestamp = nullptr;
};

struct TelemetryData {
    MetaData metadata;
    LocationData location;
    SpeedData speed;
    BlockedData blocked;
    BatteryData battery;
    LocalizationData localization;
    NavigationData navigation;
};

struct ContinuousTelemetryParams {
    int interval = 0;
};

struct LiftParams {
    float height = 0.0f;
};

struct LiftData {
    float height = 0.0f;
};

// ============================================================================
// C API function pointer types
// ============================================================================
using FnAMRConnect    = int (*)(AMRType, const char*);
using FnAMRDisconnect = int (*)(AMRType, const char*);
using FnRelocation    = int (*)(AMRType, const char*, const RelocParams*);
using FnTranslation   = int (*)(AMRType, const char*, const TranslationParams*);
using FnRotation      = int (*)(AMRType, const char*, const RotationParams*);
using FnGoTo          = int (*)(AMRType, const char*, const GoToParams*);
using FnPause         = int (*)(AMRType, const char*);
using FnResume        = int (*)(AMRType, const char*);
using FnCancel        = int (*)(AMRType, const char*);
using FnStop          = int (*)(AMRType, const char*, const StopParams*);
using FnRobotLocalizationStatus = int (*)(AMRType, const char*, LocalizationData*);
using FnRobotNavigationStatus   = int (*)(AMRType, const char*, NavigationData*);
using FnRobotLocationStatus     = int (*)(AMRType, const char*, LocationData*);
using FnRobotSpeedStatus        = int (*)(AMRType, const char*, SpeedData*);
using FnRobotBlockedStatus      = int (*)(AMRType, const char*, BlockedData*);
using FnRobotBatteryStatus      = int (*)(AMRType, const char*, BatteryData*);
using FnStartContinuousTelemetry = int (*)(AMRType, const char*, ContinuousTelemetryParams*);
using FnReadContinuousTelemetry  = int (*)(AMRType, const char*, TelemetryData*);
using FnStopContinuousTelemetry  = int (*)(AMRType, const char*);
using FnLiftHeight    = int (*)(AMRType, const char*, const LiftParams*);
using FnStopLift      = int (*)(AMRType, const char*);
using FnRobotLiftStatus = int (*)(AMRType, const char*, LiftData*);

// ============================================================================
// AMRApi — runtime DLL loading singleton
// ============================================================================
class AMRApi {
public:
    static AMRApi& instance() {
        static AMRApi inst;
        return inst;
    }

    bool load(const std::string& dll_path = "CyAMR.dll") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hDll_) return true;

        hDll_ = LoadLibraryA(dll_path.c_str());
        if (!hDll_) {
            std::cerr << "[AMRApi] LoadLibrary failed for " << dll_path
                      << " (error " << GetLastError() << ")" << std::endl;
            return false;
        }

        fnConnect    = reinterpret_cast<FnAMRConnect>(GetProcAddress(hDll_, "Connect"));
        fnDisconnect = reinterpret_cast<FnAMRDisconnect>(GetProcAddress(hDll_, "Disconnect"));
        fnRelocation = reinterpret_cast<FnRelocation>(GetProcAddress(hDll_, "Relocation"));
        fnTranslation = reinterpret_cast<FnTranslation>(GetProcAddress(hDll_, "Translation"));
        fnRotation   = reinterpret_cast<FnRotation>(GetProcAddress(hDll_, "Rotation"));
        fnGoTo       = reinterpret_cast<FnGoTo>(GetProcAddress(hDll_, "GoTo"));
        fnPause      = reinterpret_cast<FnPause>(GetProcAddress(hDll_, "Pause"));
        fnResume     = reinterpret_cast<FnResume>(GetProcAddress(hDll_, "Resume"));
        fnCancel     = reinterpret_cast<FnCancel>(GetProcAddress(hDll_, "Cancel"));
        fnStop       = reinterpret_cast<FnStop>(GetProcAddress(hDll_, "Stop"));
        fnRobotLocalizationStatus = reinterpret_cast<FnRobotLocalizationStatus>(GetProcAddress(hDll_, "RobotLocalizationStatus"));
        fnRobotNavigationStatus   = reinterpret_cast<FnRobotNavigationStatus>(GetProcAddress(hDll_, "RobotNavigationStatus"));
        fnRobotLocationStatus     = reinterpret_cast<FnRobotLocationStatus>(GetProcAddress(hDll_, "RobotLocationStatus"));
        fnRobotSpeedStatus        = reinterpret_cast<FnRobotSpeedStatus>(GetProcAddress(hDll_, "RobotSpeedStatus"));
        fnRobotBlockedStatus      = reinterpret_cast<FnRobotBlockedStatus>(GetProcAddress(hDll_, "RobotBlockedStatus"));
        fnRobotBatteryStatus      = reinterpret_cast<FnRobotBatteryStatus>(GetProcAddress(hDll_, "RobotBatteryStatus"));
        fnStartContinuousTelemetry = reinterpret_cast<FnStartContinuousTelemetry>(GetProcAddress(hDll_, "StartContinuousTelemetry"));
        fnReadContinuousTelemetry  = reinterpret_cast<FnReadContinuousTelemetry>(GetProcAddress(hDll_, "ReadContinuousTelemetry"));
        fnStopContinuousTelemetry  = reinterpret_cast<FnStopContinuousTelemetry>(GetProcAddress(hDll_, "StopContinuousTelemetry"));
        fnLiftHeight = reinterpret_cast<FnLiftHeight>(GetProcAddress(hDll_, "LiftHeight"));
        fnStopLift   = reinterpret_cast<FnStopLift>(GetProcAddress(hDll_, "StopLift"));
        fnRobotLiftStatus = reinterpret_cast<FnRobotLiftStatus>(GetProcAddress(hDll_, "RobotLiftStatus"));

        if (!fnConnect || !fnDisconnect || !fnGoTo || !fnStop ||
            !fnRobotNavigationStatus || !fnRobotLocationStatus) {
            std::cerr << "[AMRApi] Failed to resolve core function pointers" << std::endl;
            FreeLibrary(hDll_);
            hDll_ = nullptr;
            return false;
        }

        std::cout << "[AMRApi] DLL loaded: " << dll_path << std::endl;
        return true;
    }

    void unload() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (hDll_) { FreeLibrary(hDll_); hDll_ = nullptr; }
    }

    bool isLoaded() const { std::lock_guard<std::mutex> lock(mutex_); return hDll_ != nullptr; }

    // Wrapper methods
    int connect(AMRType t, const char* ip)     { return fnConnect ? fnConnect(t, ip) : -1; }
    int disconnect(AMRType t, const char* ip)  { return fnDisconnect ? fnDisconnect(t, ip) : -1; }
    int relocation(AMRType t, const char* ip, const RelocParams* p)       { return fnRelocation ? fnRelocation(t, ip, p) : -1; }
    int translation(AMRType t, const char* ip, const TranslationParams* p) { return fnTranslation ? fnTranslation(t, ip, p) : -1; }
    int rotation(AMRType t, const char* ip, const RotationParams* p)      { return fnRotation ? fnRotation(t, ip, p) : -1; }
    int goTo(AMRType t, const char* ip, const GoToParams* p)              { return fnGoTo ? fnGoTo(t, ip, p) : -1; }
    int pause(AMRType t, const char* ip)       { return fnPause ? fnPause(t, ip) : -1; }
    int resume(AMRType t, const char* ip)      { return fnResume ? fnResume(t, ip) : -1; }
    int cancel(AMRType t, const char* ip)      { return fnCancel ? fnCancel(t, ip) : -1; }
    int stop(AMRType t, const char* ip, const StopParams* p)              { return fnStop ? fnStop(t, ip, p) : -1; }
    int robotLocalizationStatus(AMRType t, const char* ip, LocalizationData* d) { return fnRobotLocalizationStatus ? fnRobotLocalizationStatus(t, ip, d) : -1; }
    int robotNavigationStatus(AMRType t, const char* ip, NavigationData* d)     { return fnRobotNavigationStatus ? fnRobotNavigationStatus(t, ip, d) : -1; }
    int robotLocationStatus(AMRType t, const char* ip, LocationData* d)         { return fnRobotLocationStatus ? fnRobotLocationStatus(t, ip, d) : -1; }
    int robotBatteryStatus(AMRType t, const char* ip, BatteryData* d)           { return fnRobotBatteryStatus ? fnRobotBatteryStatus(t, ip, d) : -1; }
    int startContinuousTelemetry(AMRType t, const char* ip, ContinuousTelemetryParams* p) { return fnStartContinuousTelemetry ? fnStartContinuousTelemetry(t, ip, p) : -1; }
    int readContinuousTelemetry(AMRType t, const char* ip, TelemetryData* d)  { return fnReadContinuousTelemetry ? fnReadContinuousTelemetry(t, ip, d) : -1; }
    int stopContinuousTelemetry(AMRType t, const char* ip)                    { return fnStopContinuousTelemetry ? fnStopContinuousTelemetry(t, ip) : -1; }
    int liftHeight(AMRType t, const char* ip, const LiftParams* p)            { return fnLiftHeight ? fnLiftHeight(t, ip, p) : -1; }
    int stopLift(AMRType t, const char* ip)    { return fnStopLift ? fnStopLift(t, ip) : -1; }

private:
    AMRApi() = default;
    ~AMRApi() = default;
    AMRApi(const AMRApi&) = delete;
    AMRApi& operator=(const AMRApi&) = delete;

    HMODULE hDll_ = nullptr;
    mutable std::mutex mutex_;

    FnAMRConnect    fnConnect = nullptr;
    FnAMRDisconnect fnDisconnect = nullptr;
    FnRelocation    fnRelocation = nullptr;
    FnTranslation   fnTranslation = nullptr;
    FnRotation      fnRotation = nullptr;
    FnGoTo          fnGoTo = nullptr;
    FnPause         fnPause = nullptr;
    FnResume        fnResume = nullptr;
    FnCancel        fnCancel = nullptr;
    FnStop          fnStop = nullptr;
    FnRobotLocalizationStatus fnRobotLocalizationStatus = nullptr;
    FnRobotNavigationStatus   fnRobotNavigationStatus = nullptr;
    FnRobotLocationStatus     fnRobotLocationStatus = nullptr;
    FnRobotSpeedStatus        fnRobotSpeedStatus = nullptr;
    FnRobotBlockedStatus      fnRobotBlockedStatus = nullptr;
    FnRobotBatteryStatus      fnRobotBatteryStatus = nullptr;
    FnStartContinuousTelemetry fnStartContinuousTelemetry = nullptr;
    FnReadContinuousTelemetry  fnReadContinuousTelemetry = nullptr;
    FnStopContinuousTelemetry  fnStopContinuousTelemetry = nullptr;
    FnLiftHeight    fnLiftHeight = nullptr;
    FnStopLift      fnStopLift = nullptr;
    FnRobotLiftStatus fnRobotLiftStatus = nullptr;
};

inline AMRType parseAMRType(const std::string& s) {
    if (s == "SEER") return AMRType::SEER;
    return AMRType::UNKNOWN;
}

// ============================================================================
// BT Nodes
// ============================================================================

class ConnectAMR : public BT::SyncActionNode {
public:
    ConnectAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::InputPort<std::string>("dll_path", "CyAMR.dll", "Path to AMR DLL") };
    }
    BT::NodeStatus tick() override;
};

class DisconnectAMR : public BT::SyncActionNode {
public:
    DisconnectAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type") };
    }
    BT::NodeStatus tick() override;
};

class RelocateAMR : public BT::SyncActionNode {
public:
    RelocateAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::InputPort<bool>("auto_reloc", true, "Auto relocate"),
                 BT::InputPort<double>("x", 0.0, "X coordinate"),
                 BT::InputPort<double>("y", 0.0, "Y coordinate"),
                 BT::InputPort<double>("angle", 0.0, "Angle (rad)"),
                 BT::InputPort<double>("area_radius", 0.0, "Search radius") };
    }
    BT::NodeStatus tick() override;
};

class PauseAMR : public BT::SyncActionNode {
public:
    PauseAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type") };
    }
    BT::NodeStatus tick() override;
};

class ResumeAMR : public BT::SyncActionNode {
public:
    ResumeAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type") };
    }
    BT::NodeStatus tick() override;
};

class CancelAMR : public BT::SyncActionNode {
public:
    CancelAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type") };
    }
    BT::NodeStatus tick() override;
};

class StopAMR : public BT::SyncActionNode {
public:
    StopAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::InputPort<bool>("emergency_stop", true, "Emergency stop") };
    }
    BT::NodeStatus tick() override;
};

class GetAMRLocationStatus : public BT::SyncActionNode {
public:
    GetAMRLocationStatus(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::OutputPort<float>("x", "X position (m)"),
                 BT::OutputPort<float>("y", "Y position (m)"),
                 BT::OutputPort<float>("angle", "Orientation angle (rad)"),
                 BT::OutputPort<float>("confidence", "Localization confidence") };
    }
    BT::NodeStatus tick() override;
};

class GetAMRNavigationStatus : public BT::SyncActionNode {
public:
    GetAMRNavigationStatus(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::OutputPort<int>("task_status", "Task status"),
                 BT::OutputPort<int>("task_type", "Task type") };
    }
    BT::NodeStatus tick() override;
};

class GetAMRBatteryStatus : public BT::SyncActionNode {
public:
    GetAMRBatteryStatus(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::OutputPort<float>("battery_level", "Battery level"),
                 BT::OutputPort<bool>("charging", "Charging status"),
                 BT::OutputPort<float>("voltage", "Battery voltage (V)") };
    }
    BT::NodeStatus tick() override;
};

class GetAMRLocalizationStatus : public BT::SyncActionNode {
public:
    GetAMRLocalizationStatus(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::OutputPort<int>("reloc_status", "Relocalization status") };
    }
    BT::NodeStatus tick() override;
};

class StartContinuousTelemetryAMR : public BT::SyncActionNode {
public:
    StartContinuousTelemetryAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::InputPort<int>("interval_ms", 100, "Telemetry interval (ms)") };
    }
    BT::NodeStatus tick() override;
};

class ReadContinuousTelemetryAMR : public BT::SyncActionNode {
public:
    ReadContinuousTelemetryAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::OutputPort<float>("x", "X position (m)"),
                 BT::OutputPort<float>("y", "Y position (m)"),
                 BT::OutputPort<float>("angle", "Orientation angle (rad)"),
                 BT::OutputPort<float>("vel_x", "Velocity X (m/s)"),
                 BT::OutputPort<float>("vel_y", "Velocity Y (m/s)"),
                 BT::OutputPort<float>("battery_level", "Battery level"),
                 BT::OutputPort<int>("task_status", "Navigation task status"),
                 BT::OutputPort<bool>("blocked", "Whether robot is blocked") };
    }
    BT::NodeStatus tick() override;
};

class StopContinuousTelemetryAMR : public BT::SyncActionNode {
public:
    StopContinuousTelemetryAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type") };
    }
    BT::NodeStatus tick() override;
};

class LiftAMR : public BT::SyncActionNode {
public:
    LiftAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type"),
                 BT::InputPort<float>("height", "Target lift height (m)") };
    }
    BT::NodeStatus tick() override;
};

class StopLiftAMR : public BT::SyncActionNode {
public:
    StopLiftAMR(const std::string& n, const BT::NodeConfig& c) : BT::SyncActionNode(n, c) {}
    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("ip", "AMR IP address"),
                 BT::InputPort<std::string>("amr_type", "SEER", "AMR type") };
    }
    BT::NodeStatus tick() override;
};

// ============================================================================
// Registration
// ============================================================================
inline void registerAMRNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ConnectAMR>("ConnectAMR");
    factory.registerNodeType<DisconnectAMR>("DisconnectAMR");
    factory.registerNodeType<RelocateAMR>("RelocateAMR");
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
