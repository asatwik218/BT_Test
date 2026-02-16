#include "motor_nodes.hpp"

#include <cmath>
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

// ============================================================================
// SetMotorAsync Implementation
// ============================================================================

BT::NodeStatus SetMotorAsync::onStart() {
    auto idx = getInput<int>("motor_index");
    auto pos = getInput<int32_t>("target_position");
    if (!idx || !pos) {
        std::cerr << "[SetMotorAsync] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* motor = MotorManager::instance().getMotor(idx.value());
    if (!motor) {
        std::cerr << "[SetMotorAsync] Motor index out of range: " << idx.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    motor_index_ = idx.value();
    motor_ = *motor;
    target_position_ = pos.value();
    tolerance_ = getInput<int32_t>("position_tolerance").value_or(100);

    StMotorInput input{};
    input.targetPosition = target_position_;
    input.profileAcceleration = getInput<uint32_t>("profile_acceleration").value_or(1000);
    input.profileDeceleration = getInput<uint32_t>("profile_deceleration").value_or(1000);
    input.profileVelocity = getInput<uint32_t>("profile_velocity").value_or(20);
    bool relative = getInput<bool>("relative").value_or(false);

    int status = MotorManager::instance().setMotor(motor_, input, relative);
    if (status != 0) {
        std::cerr << "[SetMotorAsync] Failed to send command, status=" << status << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetMotorAsync] Motor " << motor_index_ << " -> " << target_position_
              << " (tolerance=" << tolerance_ << ")" << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMotorAsync::onRunning() {
    StMotorOutput output{};
    int status = MotorManager::instance().getMotorStatus(motor_, output);
    if (status != 0) {
        std::cerr << "[SetMotorAsync] Failed to get status" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    if (output.errorCode != 0) {
        std::cerr << "[SetMotorAsync] Motor " << motor_index_ << " error: 0x"
                  << std::hex << output.errorCode << std::dec << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    int32_t error = std::abs(output.actualPosition - target_position_);
    if (error <= tolerance_) {
        std::cout << "[SetMotorAsync] Motor " << motor_index_ << " reached target, pos="
                  << output.actualPosition << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

void SetMotorAsync::onHalted() {
    std::cout << "[SetMotorAsync] HALTED - stopping motor " << motor_index_ << std::endl;
    MotorManager::instance().stopMotor(motor_);
}

// ============================================================================
// SetMotorsAsync Implementation
// ============================================================================

BT::NodeStatus SetMotorsAsync::onStart() {
    auto i1 = getInput<int>("motor_index_1");
    auto p1 = getInput<int32_t>("target_position_1");
    auto i2 = getInput<int>("motor_index_2");
    auto p2 = getInput<int32_t>("target_position_2");

    if (!i1 || !p1 || !i2 || !p2) {
        std::cerr << "[SetMotorsAsync] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const StMotor* m1 = MotorManager::instance().getMotor(i1.value());
    const StMotor* m2 = MotorManager::instance().getMotor(i2.value());
    if (!m1 || !m2) {
        std::cerr << "[SetMotorsAsync] Motor index out of range" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    idx1_ = i1.value(); idx2_ = i2.value();
    motor1_ = *m1; motor2_ = *m2;
    target1_ = p1.value(); target2_ = p2.value();
    tolerance_ = getInput<int32_t>("position_tolerance").value_or(100);

    StMotorInput input1{};
    input1.targetPosition = target1_;
    input1.profileAcceleration = getInput<uint32_t>("profile_acceleration_1").value_or(1000);
    input1.profileDeceleration = getInput<uint32_t>("profile_deceleration_1").value_or(1000);
    input1.profileVelocity = getInput<uint32_t>("profile_velocity_1").value_or(20);

    StMotorInput input2{};
    input2.targetPosition = target2_;
    input2.profileAcceleration = getInput<uint32_t>("profile_acceleration_2").value_or(1000);
    input2.profileDeceleration = getInput<uint32_t>("profile_deceleration_2").value_or(1000);
    input2.profileVelocity = getInput<uint32_t>("profile_velocity_2").value_or(20);

    int s1 = MotorManager::instance().setMotor(motor1_, input1, false);
    int s2 = MotorManager::instance().setMotor(motor2_, input2, false);
    if (s1 != 0 || s2 != 0) {
        std::cerr << "[SetMotorsAsync] Failed to send commands" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetMotorsAsync] Motor " << idx1_ << " -> " << target1_
              << ", Motor " << idx2_ << " -> " << target2_ << std::endl;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SetMotorsAsync::onRunning() {
    StMotorOutput out1{}, out2{};
    int s1 = MotorManager::instance().getMotorStatus(motor1_, out1);
    int s2 = MotorManager::instance().getMotorStatus(motor2_, out2);
    if (s1 != 0 || s2 != 0) {
        std::cerr << "[SetMotorsAsync] Failed to get status" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    if (out1.errorCode != 0 || out2.errorCode != 0) {
        std::cerr << "[SetMotorsAsync] Motor error detected" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    bool m1_done = std::abs(out1.actualPosition - target1_) <= tolerance_;
    bool m2_done = std::abs(out2.actualPosition - target2_) <= tolerance_;

    if (m1_done && m2_done) {
        std::cout << "[SetMotorsAsync] Both motors reached targets" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void SetMotorsAsync::onHalted() {
    std::cout << "[SetMotorsAsync] HALTED - stopping both motors" << std::endl;
    MotorManager::instance().stopMotor(motor1_);
    MotorManager::instance().stopMotor(motor2_);
}

} // namespace motor_bt
