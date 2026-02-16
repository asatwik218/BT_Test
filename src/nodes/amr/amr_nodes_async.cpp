#include "amr_nodes_async.hpp"

namespace amr_bt {

// ============================================================================
// GoToAMRAsync
// ============================================================================

BT::NodeStatus GoToAMRAsync::onStart() {
    auto ip = getInput<std::string>("ip");
    auto dest_id = getInput<std::string>("dest_id");

    if (!ip || !dest_id) {
        std::cerr << "[GoToAMRAsync] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    ip_ = ip.value();
    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    amr_type_ = parseAMRType(type_str);

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

    int status = AMRApi::instance().goTo(amr_type_, ip_.c_str(), &params);
    if (status != 0) {
        std::cerr << "[GoToAMRAsync] Failed to send GoTo, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[GoToAMRAsync] Navigating to '" << dest_id.value()
              << "', waiting for completion..." << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoToAMRAsync::onRunning() {
    NavigationData nav{};
    int status = AMRApi::instance().robotNavigationStatus(amr_type_, ip_.c_str(), &nav);
    if (status != 0) {
        std::cerr << "[GoToAMRAsync] Failed to get navigation status" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // task_status == 4 means navigation completed
    if (nav.task_status == 4) {
        std::cout << "[GoToAMRAsync] Navigation completed!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    // Check for blocked state
    BlockedData blocked{};
    if (AMRApi::instance().fnRobotBlockedStatus) {
        // Note: fnRobotBlockedStatus is private, so use the wrapper if available
        // For now, we rely on navigation status only
    }

    return BT::NodeStatus::RUNNING;
}

void GoToAMRAsync::onHalted() {
    std::cout << "[GoToAMRAsync] HALTED - stopping AMR" << std::endl;
    StopParams params{};
    params.output_emergency_stop = true;
    AMRApi::instance().stop(amr_type_, ip_.c_str(), &params);
}

// ============================================================================
// TranslateAMRAsync
// ============================================================================

BT::NodeStatus TranslateAMRAsync::onStart() {
    auto ip = getInput<std::string>("ip");
    auto distance = getInput<float>("distance");

    if (!ip || !distance) {
        std::cerr << "[TranslateAMRAsync] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    ip_ = ip.value();
    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    amr_type_ = parseAMRType(type_str);

    TranslationParams params{};
    params.distance = distance.value();
    params.vel_x = getInput<float>("vel_x").value_or(0.0f);
    params.vel_y = getInput<float>("vel_y").value_or(0.0f);

    int status = AMRApi::instance().translation(amr_type_, ip_.c_str(), &params);
    if (status != 0) {
        std::cerr << "[TranslateAMRAsync] Failed to send translation, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[TranslateAMRAsync] Translating " << params.distance
              << "m, waiting for completion..." << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TranslateAMRAsync::onRunning() {
    NavigationData nav{};
    int status = AMRApi::instance().robotNavigationStatus(amr_type_, ip_.c_str(), &nav);
    if (status != 0) {
        std::cerr << "[TranslateAMRAsync] Failed to get navigation status" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    if (nav.task_status == 4) {
        std::cout << "[TranslateAMRAsync] Translation completed!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void TranslateAMRAsync::onHalted() {
    std::cout << "[TranslateAMRAsync] HALTED - stopping AMR" << std::endl;
    StopParams params{};
    params.output_emergency_stop = true;
    AMRApi::instance().stop(amr_type_, ip_.c_str(), &params);
}

// ============================================================================
// RotateAMRAsync
// ============================================================================

BT::NodeStatus RotateAMRAsync::onStart() {
    auto ip = getInput<std::string>("ip");
    auto angle = getInput<float>("angle");

    if (!ip || !angle) {
        std::cerr << "[RotateAMRAsync] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    ip_ = ip.value();
    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    amr_type_ = parseAMRType(type_str);

    RotationParams params{};
    params.angle = angle.value();
    params.ang_vel = getInput<float>("ang_vel").value_or(0.0f);

    int status = AMRApi::instance().rotation(amr_type_, ip_.c_str(), &params);
    if (status != 0) {
        std::cerr << "[RotateAMRAsync] Failed to send rotation, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[RotateAMRAsync] Rotating " << params.angle
              << " rad, waiting for completion..." << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RotateAMRAsync::onRunning() {
    NavigationData nav{};
    int status = AMRApi::instance().robotNavigationStatus(amr_type_, ip_.c_str(), &nav);
    if (status != 0) {
        std::cerr << "[RotateAMRAsync] Failed to get navigation status" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    if (nav.task_status == 4) {
        std::cout << "[RotateAMRAsync] Rotation completed!" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void RotateAMRAsync::onHalted() {
    std::cout << "[RotateAMRAsync] HALTED - stopping AMR" << std::endl;
    StopParams params{};
    params.output_emergency_stop = true;
    AMRApi::instance().stop(amr_type_, ip_.c_str(), &params);
}

} // namespace amr_bt
