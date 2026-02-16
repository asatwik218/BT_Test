#include "monitoring_nodes.hpp"

namespace monitoring_bt {

// ============================================================================
// CheckMotorFaults
// ============================================================================

BT::NodeStatus CheckMotorFaults::tick() {
    int num_motors = getInput<int>("num_motors").value_or(2);

    for (int i = 0; i < num_motors; ++i) {
        const motor_bt::StMotor* motor = motor_bt::MotorManager::instance().getMotor(i);
        if (!motor) continue;

        motor_bt::StMotorOutput output{};
        int status = motor_bt::MotorManager::instance().getMotorStatus(*motor, output);
        if (status != 0) continue; // Skip if can't read status

        if (output.errorCode != 0) {
            std::cerr << "[CheckMotorFaults] Motor " << i << " fault: 0x"
                      << std::hex << output.errorCode << std::dec << std::endl;
            setOutput("fault_motor_index", i);
            setOutput("fault_error_code", static_cast<int>(output.errorCode));
            return BT::NodeStatus::FAILURE;
        }
    }

    setOutput("fault_motor_index", -1);
    setOutput("fault_error_code", 0);
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// CheckRobotFaults
// ============================================================================

BT::NodeStatus CheckRobotFaults::tick() {
    auto serial = getInput<std::string>("serial");
    if (!serial) {
        std::cerr << "[CheckRobotFaults] Missing 'serial'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        auto robot = flexiv_bt::RobotManager::instance().get(serial.value());
        if (!robot) {
            std::cerr << "[CheckRobotFaults] Robot not found: " << serial.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        if (robot->fault()) {
            std::cerr << "[CheckRobotFaults] Robot " << serial.value() << " is in FAULT state" << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "[CheckRobotFaults] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================================
// CheckAMRFaults
// ============================================================================

BT::NodeStatus CheckAMRFaults::tick() {
    auto ip = getInput<std::string>("ip");
    if (!ip) {
        std::cerr << "[CheckAMRFaults] Missing 'ip'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto type_str = getInput<std::string>("amr_type").value_or("SEER");
    amr_bt::AMRType type = amr_bt::parseAMRType(type_str);
    float min_battery = getInput<float>("min_battery_level").value_or(10.0f);

    // Check battery
    amr_bt::BatteryData battery{};
    int bat_status = amr_bt::AMRApi::instance().robotBatteryStatus(type, ip.value().c_str(), &battery);
    if (bat_status == 0) {
        setOutput("battery_level", battery.battery_level);
        if (battery.battery_level < min_battery && battery.battery_level > 0.0f) {
            std::cerr << "[CheckAMRFaults] Battery critical: " << battery.battery_level << "%" << std::endl;
            return BT::NodeStatus::FAILURE;
        }
    }

    // Check blocked state via navigation status (blocked info not directly accessible)
    // We use location status as a proxy — if it fails, AMR may be unreachable
    amr_bt::LocationData loc{};
    int loc_status = amr_bt::AMRApi::instance().robotLocationStatus(type, ip.value().c_str(), &loc);
    if (loc_status != 0) {
        std::cerr << "[CheckAMRFaults] Cannot reach AMR at " << ip.value() << std::endl;
        setOutput("is_blocked", true);
        return BT::NodeStatus::FAILURE;
    }

    setOutput("is_blocked", false);
    return BT::NodeStatus::SUCCESS;
}

} // namespace monitoring_bt
