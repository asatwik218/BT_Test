#include "amr_nodes.hpp"

#include <iostream>

namespace amr_bt {

// ============================================================================
// ConnectAMR Implementation
// ============================================================================

BT::NodeStatus ConnectAMR::tick() {
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
    int status = Connect(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[ConnectAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ConnectAMR] Connected successfully" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// DisconnectAMR Implementation
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
    int status = Disconnect(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[DisconnectAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[DisconnectAMR] Disconnected" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// RelocateAMR Implementation
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

    int status = Relocation(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[RelocateAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[RelocateAMR] Relocation command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// TranslateAMR Implementation
// ============================================================================

BT::NodeStatus TranslateAMR::tick() {
    auto ip = getInput<std::string>("ip");
    auto distance = getInput<float>("distance");

    if (!ip) {
        std::cerr << "[TranslateAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!distance) {
        std::cerr << "[TranslateAMR] Missing 'distance': " << distance.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    TranslationParams params{};
    params.distance = distance.value();
    params.vel_x = getInput<float>("vel_x").value_or(0.0f);
    params.vel_y = getInput<float>("vel_y").value_or(0.0f);

    std::cout << "[TranslateAMR] Translating " << params.distance << "m" << std::endl;
    int status = Translation(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[TranslateAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[TranslateAMR] Translation command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// RotateAMR Implementation
// ============================================================================

BT::NodeStatus RotateAMR::tick() {
    auto ip = getInput<std::string>("ip");
    auto angle = getInput<float>("angle");

    if (!ip) {
        std::cerr << "[RotateAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!angle) {
        std::cerr << "[RotateAMR] Missing 'angle': " << angle.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    RotationParams params{};
    params.angle = angle.value();
    params.ang_vel = getInput<float>("ang_vel").value_or(0.0f);

    std::cout << "[RotateAMR] Rotating " << params.angle << " rad" << std::endl;
    int status = Rotation(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[RotateAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[RotateAMR] Rotation command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GoToAMR Implementation
// ============================================================================

BT::NodeStatus GoToAMR::tick() {
    auto ip = getInput<std::string>("ip");
    auto dest_id = getInput<std::string>("dest_id");

    if (!ip) {
        std::cerr << "[GoToAMR] Missing 'ip': " << ip.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!dest_id) {
        std::cerr << "[GoToAMR] Missing 'dest_id': " << dest_id.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    AMRType type = parseAMRType(type_str);

    auto source_id_str = getInput<std::string>("source_id").value_or("SELF_POSITION");
    auto movement_mode_str = getInput<std::string>("movement_mode").value_or("forward");

    GoToParams params{};
    params.source_id = source_id_str.c_str();
    params.dest_id = dest_id.value().c_str();
    params.movement_mode = movement_mode_str.c_str();
    params.arrival_angle = getInput<float>("arrival_angle").value_or(0.0f);
    params.max_vel = getInput<float>("max_vel").value_or(0.0f);
    params.max_ang_vel = getInput<float>("max_ang_vel").value_or(0.0f);
    params.max_acc = getInput<float>("max_acc").value_or(0.0f);
    params.max_ang_acc = getInput<float>("max_ang_acc").value_or(0.0f);

    std::cout << "[GoToAMR] Navigating from '" << source_id_str
              << "' to '" << dest_id.value() << "'" << std::endl;

    int status = GoTo(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[GoToAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[GoToAMR] Navigation command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// PauseAMR Implementation
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
    int status = Pause(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[PauseAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[PauseAMR] Paused" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ResumeAMR Implementation
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
    int status = Resume(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[ResumeAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ResumeAMR] Resumed" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// CancelAMR Implementation
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
    int status = Cancel(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[CancelAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[CancelAMR] Canceled" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopAMR Implementation
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
    int status = Stop(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[StopAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopAMR] Stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetAMRLocationStatus Implementation
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
    int status = RobotLocationStatus(type, ip.value().c_str(), &data);
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
// GetAMRNavigationStatus Implementation
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
    int status = RobotNavigationStatus(type, ip.value().c_str(), &data);
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
// GetAMRBatteryStatus Implementation
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
    int status = RobotBatteryStatus(type, ip.value().c_str(), &data);
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
// GetAMRLocalizationStatus Implementation
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
    int status = RobotLocalizationStatus(type, ip.value().c_str(), &data);
    if (status != 0) {
        std::cerr << "[GetAMRLocalizationStatus] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("reloc_status", data.reloc_status);

    std::cout << "[GetAMRLocalizationStatus] reloc_status=" << data.reloc_status << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StartContinuousTelemetryAMR Implementation
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
    int status = StartContinuousTelemetry(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[StartContinuousTelemetryAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StartContinuousTelemetryAMR] Telemetry started" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ReadContinuousTelemetryAMR Implementation
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
    int status = ReadContinuousTelemetry(type, ip.value().c_str(), &data);
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
// StopContinuousTelemetryAMR Implementation
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
    int status = StopContinuousTelemetry(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[StopContinuousTelemetryAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopContinuousTelemetryAMR] Telemetry stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// LiftAMR Implementation
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
    int status = LiftHeight(type, ip.value().c_str(), &params);
    if (status != 0) {
        std::cerr << "[LiftAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[LiftAMR] Lift command sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopLiftAMR Implementation
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
    int status = StopLift(type, ip.value().c_str());
    if (status != 0) {
        std::cerr << "[StopLiftAMR] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopLiftAMR] Lift stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace amr_bt
