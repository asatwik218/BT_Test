#pragma once

#include "behaviortree_cpp/bt_factory.h"

#include <Windows.h>
#include <cstdint>
#include <vector>
#include <mutex>
#include <string>
#include <iostream>

namespace motor_bt {

// ============================================================================
// Struct definitions (matching CyMotorControlInterface_C.h layout exactly)
// ============================================================================
#pragma pack(push,1)
struct StMotor {
    uint32_t motor_handle;
};

struct StMotorInput {
    int32_t targetPosition;
    uint32_t profileAcceleration;
    uint32_t profileDeceleration;
    uint32_t profileVelocity;
};

struct StMotorOutput {
    uint16_t statusWord;
    int32_t actualPosition;
    int32_t velocityActualValue;
    int16_t currentActualValue;
    int32_t followingErrActualValue;
    int8_t modeOfOperationDisplay;
    uint32_t digitalInput;
    uint16_t errorCode;
};
#pragma pack(pop)

// Opaque handle — never dereferenced, only passed through to DLL
struct CyMotorControlInterface;

// ============================================================================
// C API function pointer types (from CyMotorControlInterface_C.h)
// ============================================================================
using FnGetInstance     = CyMotorControlInterface* (*)(int* status);
using FnGetMotorCount   = int (*)(CyMotorControlInterface*, int*);
using FnGetMotors       = int (*)(CyMotorControlInterface*, StMotor*, int);
using FnStartMotor      = int (*)(CyMotorControlInterface*, StMotor, StMotorInput);
using FnStopMotor       = int (*)(CyMotorControlInterface*, StMotor);
using FnMakeHome        = int (*)(CyMotorControlInterface*, StMotor);
using FnRecalibrateHome = int (*)(CyMotorControlInterface*, const StMotor&);
using FnSetMotor        = int (*)(CyMotorControlInterface*, StMotor, StMotorInput, bool);
using FnGetMotorStatus  = int (*)(CyMotorControlInterface*, StMotor, StMotorOutput*);
using FnReleaseInstance = int (*)(CyMotorControlInterface*);

// ============================================================================
// MotorManager — runtime DLL loading, no .h or .lib required
// ============================================================================
class MotorManager {
public:
    static MotorManager& instance() {
        static MotorManager inst;
        return inst;
    }

    int connect(const std::string& dll_path = "CyMotorControlInterface.dll") {
        std::lock_guard<std::mutex> lock(mutex_);
        if (interface_) {
            std::cout << "[MotorManager] Already connected" << std::endl;
            return 0;
        }

        hDll_ = LoadLibraryA(dll_path.c_str());
        if (!hDll_) {
            std::cerr << "[MotorManager] LoadLibrary failed for " << dll_path
                      << " (error " << GetLastError() << ")" << std::endl;
            return -1;
        }

        fnGetInstance     = reinterpret_cast<FnGetInstance>(GetProcAddress(hDll_, "Get_Instance"));
        fnGetMotorCount   = reinterpret_cast<FnGetMotorCount>(GetProcAddress(hDll_, "Get_Motor_Count"));
        fnGetMotors       = reinterpret_cast<FnGetMotors>(GetProcAddress(hDll_, "Get_Motors"));
        fnStartMotor      = reinterpret_cast<FnStartMotor>(GetProcAddress(hDll_, "Start_Motor"));
        fnStopMotor       = reinterpret_cast<FnStopMotor>(GetProcAddress(hDll_, "Stop_Motor"));
        fnMakeHome        = reinterpret_cast<FnMakeHome>(GetProcAddress(hDll_, "Make_Home"));
        fnRecalibrateHome = reinterpret_cast<FnRecalibrateHome>(GetProcAddress(hDll_, "Recalibrate_Home"));
        fnSetMotor        = reinterpret_cast<FnSetMotor>(GetProcAddress(hDll_, "Set_Motor"));
        fnGetMotorStatus  = reinterpret_cast<FnGetMotorStatus>(GetProcAddress(hDll_, "Get_Motor_Status"));
        fnReleaseInstance = reinterpret_cast<FnReleaseInstance>(GetProcAddress(hDll_, "Release_Instance"));

        if (!fnGetInstance || !fnGetMotorCount || !fnGetMotors ||
            !fnStartMotor || !fnStopMotor || !fnSetMotor ||
            !fnGetMotorStatus || !fnReleaseInstance) {
            std::cerr << "[MotorManager] Failed to resolve one or more function pointers" << std::endl;
            FreeLibrary(hDll_);
            hDll_ = nullptr;
            return -2;
        }

        int status = 0;
        interface_ = fnGetInstance(&status);
        if (status != 0 || !interface_) {
            std::cerr << "[MotorManager] Get_Instance failed, status=" << status << std::endl;
            FreeLibrary(hDll_);
            hDll_ = nullptr;
            return status;
        }

        int count = 0;
        status = fnGetMotorCount(interface_, &count);
        if (status != 0) {
            std::cerr << "[MotorManager] Get_Motor_Count failed, status=" << status << std::endl;
            fnReleaseInstance(interface_);
            interface_ = nullptr;
            FreeLibrary(hDll_);
            hDll_ = nullptr;
            return status;
        }

        motors_.resize(count);
        status = fnGetMotors(interface_, motors_.data(), count);
        if (status != 0) {
            std::cerr << "[MotorManager] Get_Motors failed, status=" << status << std::endl;
            fnReleaseInstance(interface_);
            interface_ = nullptr;
            motors_.clear();
            FreeLibrary(hDll_);
            hDll_ = nullptr;
            return status;
        }

        std::cout << "[MotorManager] Connected, found " << count << " motors" << std::endl;
        return 0;
    }

    void disconnect() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (interface_ && fnReleaseInstance) {
            fnReleaseInstance(interface_);
            interface_ = nullptr;
        }
        motors_.clear();
        if (hDll_) {
            FreeLibrary(hDll_);
            hDll_ = nullptr;
        }
        std::cout << "[MotorManager] Disconnected" << std::endl;
    }

    const StMotor* getMotor(int index) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (index < 0 || index >= static_cast<int>(motors_.size())) return nullptr;
        return &motors_[index];
    }

    int motorCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<int>(motors_.size());
    }

    bool isConnected() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return interface_ != nullptr;
    }

    // Wrapper methods — forward to DLL C functions
    int startMotor(StMotor motor, StMotorInput input) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interface_ || !fnStartMotor) return -1;
        return fnStartMotor(interface_, motor, input);
    }

    int stopMotor(StMotor motor) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interface_ || !fnStopMotor) return -1;
        return fnStopMotor(interface_, motor);
    }

    int makeHome(StMotor motor) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interface_ || !fnMakeHome) return -1;
        return fnMakeHome(interface_, motor);
    }

    int recalibrateHome(const StMotor& motor) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interface_ || !fnRecalibrateHome) return -1;
        return fnRecalibrateHome(interface_, motor);
    }

    int setMotor(StMotor motor, StMotorInput input, bool relative) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interface_ || !fnSetMotor) return -1;
        return fnSetMotor(interface_, motor, input, relative);
    }

    int getMotorStatus(StMotor motor, StMotorOutput& output) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!interface_ || !fnGetMotorStatus) return -1;
        return fnGetMotorStatus(interface_, motor, &output);
    }

private:
    MotorManager() = default;
    ~MotorManager() = default;
    MotorManager(const MotorManager&) = delete;
    MotorManager& operator=(const MotorManager&) = delete;

    HMODULE hDll_ = nullptr;
    CyMotorControlInterface* interface_ = nullptr;
    std::vector<StMotor> motors_;
    mutable std::mutex mutex_;

    FnGetInstance     fnGetInstance = nullptr;
    FnGetMotorCount   fnGetMotorCount = nullptr;
    FnGetMotors       fnGetMotors = nullptr;
    FnStartMotor      fnStartMotor = nullptr;
    FnStopMotor       fnStopMotor = nullptr;
    FnMakeHome        fnMakeHome = nullptr;
    FnRecalibrateHome fnRecalibrateHome = nullptr;
    FnSetMotor        fnSetMotor = nullptr;
    FnGetMotorStatus  fnGetMotorStatus = nullptr;
    FnReleaseInstance fnReleaseInstance = nullptr;
};

// ============================================================================
// Node 1: ConnectMotors
// ============================================================================
class ConnectMotors : public BT::SyncActionNode {
public:
    ConnectMotors(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("dll_path", "CyMotorControlInterface.dll", "Path to motor DLL"),
            BT::OutputPort<int>("motor_count", "Number of motors discovered")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 2: DisconnectMotors
// ============================================================================
class DisconnectMotors : public BT::SyncActionNode {
public:
    DisconnectMotors(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 3: StartMotor
// ============================================================================
class StartMotor : public BT::SyncActionNode {
public:
    StartMotor(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("motor_index", "Motor index (0-based)"),
            BT::InputPort<int32_t>("target_position", 0, "Target position"),
            BT::InputPort<uint32_t>("profile_acceleration", 1000, "Profile acceleration"),
            BT::InputPort<uint32_t>("profile_deceleration", 1000, "Profile deceleration"),
            BT::InputPort<uint32_t>("profile_velocity", 20, "Profile velocity")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 4: StopMotor
// ============================================================================
class StopMotor : public BT::SyncActionNode {
public:
    StopMotor(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("motor_index", "Motor index (0-based)") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 5: GetMotorStatus
// ============================================================================
class GetMotorStatus : public BT::SyncActionNode {
public:
    GetMotorStatus(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("motor_index", "Motor index (0-based)"),
            BT::OutputPort<int32_t>("actual_position", "Current actual position"),
            BT::OutputPort<int32_t>("velocity", "Current velocity"),
            BT::OutputPort<uint16_t>("status_word", "Status word"),
            BT::OutputPort<uint16_t>("error_code", "Error code")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 8: MakeHome
// ============================================================================
class MakeHome : public BT::SyncActionNode {
public:
    MakeHome(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("motor_index", "Motor index (0-based)") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 9: RecalibrateHome
// ============================================================================
class RecalibrateHome : public BT::SyncActionNode {
public:
    RecalibrateHome(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<int>("motor_index", "Motor index (0-based)") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// SetMotorAsync — sends command in onStart, polls position in onRunning
// ============================================================================
class SetMotorAsync : public BT::StatefulActionNode {
public:
    SetMotorAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("motor_index", "Motor index (0-based)"),
            BT::InputPort<int32_t>("target_position", "Target position in counts"),
            BT::InputPort<uint32_t>("profile_acceleration", 1000, "Profile acceleration"),
            BT::InputPort<uint32_t>("profile_deceleration", 1000, "Profile deceleration"),
            BT::InputPort<uint32_t>("profile_velocity", 20, "Profile velocity"),
            BT::InputPort<bool>("relative", false, "Use relative positioning"),
            BT::InputPort<int32_t>("position_tolerance", 100, "Position tolerance in counts")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    int motor_index_ = 0;
    StMotor motor_{};
    int32_t target_position_ = 0;
    int32_t tolerance_ = 100;
};

// ============================================================================
// SetMotorsAsync — sends two motor commands, polls both positions
// ============================================================================
class SetMotorsAsync : public BT::StatefulActionNode {
public:
    SetMotorsAsync(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("motor_index_1", "First motor index"),
            BT::InputPort<int32_t>("target_position_1", "First motor target position"),
            BT::InputPort<uint32_t>("profile_acceleration_1", 1000, "First motor acceleration"),
            BT::InputPort<uint32_t>("profile_deceleration_1", 1000, "First motor deceleration"),
            BT::InputPort<uint32_t>("profile_velocity_1", 20, "First motor velocity"),
            BT::InputPort<int>("motor_index_2", "Second motor index"),
            BT::InputPort<int32_t>("target_position_2", "Second motor target position"),
            BT::InputPort<uint32_t>("profile_acceleration_2", 1000, "Second motor acceleration"),
            BT::InputPort<uint32_t>("profile_deceleration_2", 1000, "Second motor deceleration"),
            BT::InputPort<uint32_t>("profile_velocity_2", 20, "Second motor velocity"),
            BT::InputPort<int32_t>("position_tolerance", 100, "Position tolerance in counts")
        };
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    StMotor motor1_{}, motor2_{};
    int idx1_ = 0, idx2_ = 0;
    int32_t target1_ = 0, target2_ = 0;
    int32_t tolerance_ = 100;
};

// ============================================================================
// Registration helper
// ============================================================================
inline void registerMotorNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ConnectMotors>("ConnectMotors");
    factory.registerNodeType<DisconnectMotors>("DisconnectMotors");
    factory.registerNodeType<StartMotor>("StartMotor");
    factory.registerNodeType<StopMotor>("StopMotor");
    factory.registerNodeType<GetMotorStatus>("GetMotorStatus");
    factory.registerNodeType<MakeHome>("MakeHome");
    factory.registerNodeType<RecalibrateHome>("RecalibrateHome");
    factory.registerNodeType<SetMotorAsync>("SetMotorAsync");
    factory.registerNodeType<SetMotorsAsync>("SetMotorsAsync");
}

} // namespace motor_bt
