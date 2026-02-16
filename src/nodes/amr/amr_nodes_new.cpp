#include "amr_nodes_new.hpp"

#include <iostream>

namespace amr_bt {

// ============================================================================
// ConnectAMR
// ============================================================================

BT::NodeStatus ConnectAMR::tick() {
    auto dll_path = getInput<std::string>("dll_path").value_or("CyAMR.dll");

    if (!AMRApi::instance().isLoaded()) {
        if (!AMRApi::instance().load(dll_path)) {
            std::cerr << "[ConnectAMR] Failed to load AMR DLL: " << dll_path << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[ConnectAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);
    if (type == AMRType::UNKNOWN) {
        std::cerr << "[ConnectAMR] Unknown AMR type: " << type_str << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ConnectAMR] Connecting to " << type_str << " AMR at " << ip.value() << std::endl;
    int status = AMRApi::instance().connect(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[ConnectAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ConnectAMR] Connected successfully" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// DisconnectAMR
// ============================================================================

BT::NodeStatus DisconnectAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[DisconnectAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    std::cout << "[DisconnectAMR] Disconnecting from " << ip.value() << std::endl;
    int status = AMRApi::instance().disconnect(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[DisconnectAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[DisconnectAMR] Disconnected" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// RelocateAMR
// ============================================================================

BT::NodeStatus RelocateAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[RelocateAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    RelocParams params{};
    params.auto_reloc = getInput<bool>("auto_reloc").value_or(true);
    params.x = getInput<double>("x").value_or(0.0);
    params.y = getInput<double>("y").value_or(0.0);
    params.angle = getInput<double>("angle").value_or(0.0);
    params.area_radius = getInput<double>("area_radius").value_or(0.0);

    std::cout << "[RelocateAMR] Relocating at (" << params.x << ", " << params.y
              << ") angle=" << params.angle << " auto=" << params.auto_reloc << std::endl;

    int status = AMRApi::instance().relocation(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[RelocateAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[RelocateAMR] Relocation command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// PauseAMR
// ============================================================================

BT::NodeStatus PauseAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[PauseAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    std::cout << "[PauseAMR] Pausing AMR" << std::endl;
    int status = AMRApi::instance().pause(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[PauseAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[PauseAMR] Paused" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ResumeAMR
// ============================================================================

BT::NodeStatus ResumeAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[ResumeAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    std::cout << "[ResumeAMR] Resuming AMR" << std::endl;
    int status = AMRApi::instance().resume(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[ResumeAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ResumeAMR] Resumed" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// CancelAMR
// ============================================================================

BT::NodeStatus CancelAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[CancelAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    std::cout << "[CancelAMR] Canceling AMR task" << std::endl;
    int status = AMRApi::instance().cancel(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[CancelAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[CancelAMR] Canceled" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopAMR
// ============================================================================

BT::NodeStatus StopAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[StopAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    StopParams params{};
    params.output_emergency_stop = getInput<bool>("emergency_stop").value_or(true);

    std::cout << "[StopAMR] Stopping AMR (emergency=" << params.output_emergency_stop << ")" << std::endl;
    int status = AMRApi::instance().stop(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[StopAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopAMR] Stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetAMRLocationStatus
// ============================================================================

BT::NodeStatus GetAMRLocationStatus::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[GetAMRLocationStatus] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    LocationData data{};
    int status = AMRApi::instance().robotLocationStatus(type, ip.value().c_str(), &data);
    if (status != 0) {
        std::cerr << "[GetAMRLocationStatus] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("x", data.x);
    setOutput("y", data.y);
    setOutput("angle", data.angle);
    setOutput("confidence", data.confidence);

    std::cout << "[GetAMRLocationStatus] pos=(" << data.x << ", " << data.y
              << ") angle=" << data.angle << " conf=" << data.confidence << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetAMRNavigationStatus
// ============================================================================

BT::NodeStatus GetAMRNavigationStatus::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[GetAMRNavigationStatus] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    NavigationData data{};
    int status = AMRApi::instance().robotNavigationStatus(type, ip.value().c_str(), &data);
    if (status != 0) {
        std::cerr << "[GetAMRNavigationStatus] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("task_status", data.task_status);
    setOutput("task_type", data.task_type);

    std::cout << "[GetAMRNavigationStatus] task_status=" << data.task_status
              << " task_type=" << data.task_type << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetAMRBatteryStatus
// ============================================================================

BT::NodeStatus GetAMRBatteryStatus::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[GetAMRBatteryStatus] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    BatteryData data{};
    int status = AMRApi::instance().robotBatteryStatus(type, ip.value().c_str(), &data);
    if (status != 0) {
        std::cerr << "[GetAMRBatteryStatus] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("battery_level", data.battery_level);
    setOutput("charging", data.charging);
    setOutput("voltage", data.volatage);

    std::cout << "[GetAMRBatteryStatus] level=" << data.battery_level
              << " charging=" << data.charging << " voltage=" << data.volatage << "V" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetAMRLocalizationStatus
// ============================================================================

BT::NodeStatus GetAMRLocalizationStatus::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[GetAMRLocalizationStatus] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    LocalizationData data{};
    int status = AMRApi::instance().robotLocalizationStatus(type, ip.value().c_str(), &data);
    if (status != 0) {
        std::cerr << "[GetAMRLocalizationStatus] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("reloc_status", data.reloc_status);

    std::cout << "[GetAMRLocalizationStatus] reloc_status=" << data.reloc_status << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StartContinuousTelemetryAMR
// ============================================================================

BT::NodeStatus StartContinuousTelemetryAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[StartContinuousTelemetryAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    ContinuousTelemetryParams params{};
    params.interval = getInput<int>("interval_ms").value_or(100);

    std::cout << "[StartContinuousTelemetryAMR] Starting telemetry at " << params.interval << "ms interval" << std::endl;
    int status = AMRApi::instance().startContinuousTelemetry(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[StartContinuousTelemetryAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StartContinuousTelemetryAMR] Telemetry started" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ReadContinuousTelemetryAMR
// ============================================================================

BT::NodeStatus ReadContinuousTelemetryAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[ReadContinuousTelemetryAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    TelemetryData data{};
    int status = AMRApi::instance().readContinuousTelemetry(type, ip.value().c_str(), &data);
    if (status != 0) {
        std::cerr << "[ReadContinuousTelemetryAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("x", data.location.x);
    setOutput("y", data.location.y);
    setOutput("angle", data.location.angle);
    setOutput("vel_x", data.speed.vel_x);
    setOutput("vel_y", data.speed.vel_y);
    setOutput("battery_level", data.battery.battery_level);
    setOutput("task_status", data.navigation.task_status);
    setOutput("blocked", data.blocked.blocked);

    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopContinuousTelemetryAMR
// ============================================================================

BT::NodeStatus StopContinuousTelemetryAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[StopContinuousTelemetryAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    std::cout << "[StopContinuousTelemetryAMR] Stopping telemetry" << std::endl;
    int status = AMRApi::instance().stopContinuousTelemetry(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[StopContinuousTelemetryAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopContinuousTelemetryAMR] Telemetry stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// LiftAMR
// ============================================================================

BT::NodeStatus LiftAMR::tick() {
    auto ip = getInput<std::string>("ip");
    auto height = getInput<float>("height");

    if (!ip) {
        std::cerr << "[LiftAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!height) {
        std::cerr << "[LiftAMR] Missing 'height': " << height.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    LiftParams params{};
    params.height = height.value();

    std::cout << "[LiftAMR] Lifting to " << params.height << "m" << std::endl;
    int status = AMRApi::instance().liftHeight(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[LiftAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[LiftAMR] Lift command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopLiftAMR
// ============================================================================

BT::NodeStatus StopLiftAMR::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[StopLiftAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    std::cout << "[StopLiftAMR] Stopping lift" << std::endl;
    int status = AMRApi::instance().stopLift(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[StopLiftAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopLiftAMR] Lift stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt
