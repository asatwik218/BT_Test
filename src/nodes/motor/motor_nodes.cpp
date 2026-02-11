#include "motor_nodes.hpp"

#include <iostream>

namespace motor_bt {

// ============================================================================
// ConnectMotors
// ============================================================================

BT::NodeStatus ConnectMotors::tick() {
    auto dll_path = getInput<std::string>("dll_path").value_or("CyMotorControlInterface.dll");

    int status = MotorManager::instance().connect(dll_path);
    if (status != 0) {
        std::cerr << "[ConnectMotors] Failed to connect, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    int count = MotorManager::instance().motorCount();
    setOutput("motor_count", count);

    std::cout << "[ConnectMotors] Connected, found " << count << " motors" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// DisconnectMotors
// ============================================================================

BT::NodeStatus DisconnectMotors::tick() {
    MotorManager::instance().disconnect();
    std::cout << "[DisconnectMotors] Motors disconnected" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StartMotor
// ============================================================================

BT::NodeStatus StartMotor::tick() {
    auto idx = getInput<int>("motor_index");
    if (!idx) {
        std::cerr << "[StartMotor] Missing 'motor_index': " << idx.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[StartMotor] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    StMotorInput input{};
    input.targetPosition = getInput<int32_t>("target_position").value_or(0);
    input.profileAcceleration = getInput<uint32_t>("profile_acceleration").value_or(1000);
    input.profileDeceleration = getInput<uint32_t>("profile_deceleration").value_or(1000);
    input.profileVelocity = getInput<uint32_t>("profile_velocity").value_or(20);

    std::cout << "[StartMotor] Starting motor " << idx.value() << std::endl;
    int status = MotorManager::instance().startMotor(*motor, input);
    if (status != 0) {
        std::cerr << "[StartMotor] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StartMotor] Motor " << idx.value() << " started" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopMotor
// ============================================================================

BT::NodeStatus StopMotor::tick() {
    auto idx = getInput<int>("motor_index");
    if (!idx) {
        std::cerr << "[StopMotor] Missing 'motor_index': " << idx.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[StopMotor] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopMotor] Stopping motor " << idx.value() << std::endl;
    int status = MotorManager::instance().stopMotor(*motor);
    if (status != 0) {
        std::cerr << "[StopMotor] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopMotor] Motor " << idx.value() << " stopped" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetMotor
// ============================================================================

BT::NodeStatus SetMotor::tick() {
    auto idx = getInput<int>("motor_index");
    auto pos = getInput<int32_t>("target_position");

    if (!idx) {
        std::cerr << "[SetMotor] Missing 'motor_index': " << idx.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!pos) {
        std::cerr << "[SetMotor] Missing 'target_position': " << pos.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[SetMotor] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    StMotorInput input{};
    input.targetPosition = pos.value();
    input.profileAcceleration = getInput<uint32_t>("profile_acceleration").value_or(1000);
    input.profileDeceleration = getInput<uint32_t>("profile_deceleration").value_or(1000);
    input.profileVelocity = getInput<uint32_t>("profile_velocity").value_or(20);

    bool relative = getInput<bool>("relative").value_or(false);

    std::cout << "[SetMotor] Motor " << idx.value() << " -> position=" << pos.value()
              << (relative ? " (relative)" : " (absolute)") << std::endl;

    int status = MotorManager::instance().setMotor(*motor, input, relative);
    if (status != 0) {
        std::cerr << "[SetMotor] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetMotor] Command sent successfully" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetMotors (two sequential Set_Motor calls via C API)
// ============================================================================

BT::NodeStatus SetMotors::tick() {
    auto idx1 = getInput<int>("motor_index_1");
    auto pos1 = getInput<int32_t>("target_position_1");
    auto idx2 = getInput<int>("motor_index_2");
    auto pos2 = getInput<int32_t>("target_position_2");

    if (!idx1 || !pos1 || !idx2 || !pos2) {
        std::cerr << "[SetMotors] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor1 = MotorManager::instance().getMotor(idx1.value());
    const StMotor* motor2 = MotorManager::instance().getMotor(idx2.value());
    if (!motor1 || !motor2) {
        std::cerr << "[SetMotors] Motor index out of range" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    StMotorInput input1{};
    input1.targetPosition = pos1.value();
    input1.profileAcceleration = getInput<uint32_t>("profile_acceleration_1").value_or(1000);
    input1.profileDeceleration = getInput<uint32_t>("profile_deceleration_1").value_or(1000);
    input1.profileVelocity = getInput<uint32_t>("profile_velocity_1").value_or(20);

    StMotorInput input2{};
    input2.targetPosition = pos2.value();
    input2.profileAcceleration = getInput<uint32_t>("profile_acceleration_2").value_or(1000);
    input2.profileDeceleration = getInput<uint32_t>("profile_deceleration_2").value_or(1000);
    input2.profileVelocity = getInput<uint32_t>("profile_velocity_2").value_or(20);

    std::cout << "[SetMotors] Motor " << idx1.value() << " -> " << pos1.value()
              << ", Motor " << idx2.value() << " -> " << pos2.value() << std::endl;

    int status = MotorManager::instance().setMotor(*motor1, input1, false);
    if (status != 0) {
        std::cerr << "[SetMotors] Motor 1 failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    status = MotorManager::instance().setMotor(*motor2, input2, false);
    if (status != 0) {
        std::cerr << "[SetMotors] Motor 2 failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetMotors] Commands sent" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetMotorStatus
// ============================================================================

BT::NodeStatus GetMotorStatus::tick() {
    auto idx = getInput<int>("motor_index");
    if (!idx) {
        std::cerr << "[GetMotorStatus] Missing 'motor_index': " << idx.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[GetMotorStatus] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    StMotorOutput output{};
    int status = MotorManager::instance().getMotorStatus(*motor, output);
    if (status != 0) {
        std::cerr << "[GetMotorStatus] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    setOutput("actual_position", output.actualPosition);
    setOutput("velocity", output.velocityActualValue);
    setOutput("status_word", output.statusWord);
    setOutput("error_code", output.errorCode);

    std::cout << "[GetMotorStatus] Motor " << idx.value()
              << ": pos=" << output.actualPosition
              << " vel=" << output.velocityActualValue
              << " status=0x" << std::hex << output.statusWord
              << " error=0x" << output.errorCode << std::dec << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// MakeHome
// ============================================================================

BT::NodeStatus MakeHome::tick() {
    auto idx = getInput<int>("motor_index");
    if (!idx) {
        std::cerr << "[MakeHome] Missing 'motor_index': " << idx.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[MakeHome] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[MakeHome] Setting home for motor " << idx.value() << std::endl;
    int status = MotorManager::instance().makeHome(*motor);
    if (status != 0) {
        std::cerr << "[MakeHome] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[MakeHome] Home set for motor " << idx.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// RecalibrateHome
// ============================================================================

BT::NodeStatus RecalibrateHome::tick() {
    auto idx = getInput<int>("motor_index");
    if (!idx) {
        std::cerr << "[RecalibrateHome] Missing 'motor_index': " << idx.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[RecalibrateHome] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[RecalibrateHome] Recalibrating home for motor " << idx.value() << std::endl;
    int status = MotorManager::instance().recalibrateHome(*motor);
    if (status != 0) {
        std::cerr << "[RecalibrateHome] Failed, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[RecalibrateHome] Home recalibrated for motor " << idx.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace motor_bt
