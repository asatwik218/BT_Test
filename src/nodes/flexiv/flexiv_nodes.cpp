#include "flexiv_nodes.hpp"
#include "flexiv/rdk/mode.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <thread>
#include <chrono>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace flexiv_bt {

// ============================================================================
// ConnectRobot Implementation
// ============================================================================

BT::NodeStatus ConnectRobot::tick() {
    auto serial = getInput<std::string>("serial");
    if (!serial) {
        std::cerr << "[ConnectRobot] Missing required input 'serial': " << serial.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        std::cout << "[ConnectRobot] Connecting to robot: " << serial.value() << std::endl;

        auto robot = RobotManager::instance().getOrCreate(serial.value());
        std::cout << "[ConnectRobot] Robot object created" << std::endl;

        // Enable the robot (E-stop must be released)
        std::cout << "[ConnectRobot] Enabling robot..." << std::endl;
        robot->Enable();
        std::cout << "[ConnectRobot] Robot enabled" << std::endl;

        // Wait for robot to be operational
        int timeout_count = 0;
        const int max_timeout = 300; // 30 seconds at 100ms intervals
        bool is_operational = robot->operational();

        while (!is_operational && timeout_count < max_timeout) {
            if (timeout_count % 10 == 0) {  // Print every 1 second
                std::cout << "[ConnectRobot] Waiting for robot to be operational... ("
                          << timeout_count / 10 << "s)" << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeout_count++;
            is_operational = robot->operational();
        }

        if (!is_operational) {
            std::cerr << "[ConnectRobot] Robot not operational after timeout. Status: "
                      << static_cast<int>(robot->operational_status()) << std::endl;
            RobotManager::instance().remove(serial.value());
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[ConnectRobot] Successfully connected to: " << serial.value() << std::endl;
        return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "[ConnectRobot] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    } catch (...) {
        std::cerr << "[ConnectRobot] Unknown exception caught" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================================
// StopRobot Implementation
// ============================================================================

BT::NodeStatus StopRobot::tick() {
    auto serial = getInput<std::string>("serial");
    if (!serial) {
        std::cerr << "[StopRobot] Missing 'serial': " << serial.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    try {
        auto robot = RobotManager::instance().get(serial.value());
        if (!robot) {
            std::cerr << "[StopRobot] Robot not found: " << serial.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::cout << "[StopRobot] Stopping robot: " << serial.value() << std::endl;
        robot->Stop();

        std::cout << "[StopRobot] Robot stopped successfully" << std::endl;
        return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "[StopRobot] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================================
// DisconnectRobot Implementation
// ============================================================================

BT::NodeStatus DisconnectRobot::tick() {
    auto serial = getInput<std::string>("serial");
    if (!serial) {
        std::cerr << "[DisconnectRobot] Missing 'serial': " << serial.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[DisconnectRobot] Disconnecting robot: " << serial.value() << std::endl;

    // Remove from manager - shared_ptr destructor will close connection
    RobotManager::instance().remove(serial.value());

    std::cout << "[DisconnectRobot] Robot disconnected: " << serial.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// Helper functions for MoveRobotAsync
// ============================================================================

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double MM2M = 0.001;
constexpr double M2MM = 1000.0;

static std::array<double, 4> eulerToQuat(const std::array<double, 3>& euler_rad) {
    Eigen::Quaterniond q =
        Eigen::AngleAxisd(euler_rad[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(euler_rad[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(euler_rad[0], Eigen::Vector3d::UnitX());
    return {q.w(), q.x(), q.y(), q.z()};
}

static double posDist(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    double dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

static double orientDistDeg(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
    double dot = std::abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
    dot = std::min(1.0, dot);
    return 2.0 * std::acos(dot) * (180.0 / M_PI);
}

// ============================================================================
// MoveRobotAsync Implementation
// ============================================================================

BT::NodeStatus MoveRobotAsync::onStart() {
    auto serial = getInput<std::string>("serial");
    auto position_mm = getInput<std::array<double, 3>>("position_mm");
    auto orientation_deg = getInput<std::array<double, 3>>("orientation_deg");

    if (!serial || !position_mm || !orientation_deg) {
        std::cerr << "[MoveRobotAsync] Missing required inputs" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    serial_ = serial.value();
    target_pos_mm_ = position_mm.value();
    pos_tol_mm_ = getInput<double>("position_tolerance_mm").value_or(2.0);
    orient_tol_deg_ = getInput<double>("orientation_tolerance_deg").value_or(2.0);

    std::array<double, 3> euler_rad = {
        orientation_deg.value()[0] * DEG2RAD,
        orientation_deg.value()[1] * DEG2RAD,
        orientation_deg.value()[2] * DEG2RAD
    };
    target_quat_ = eulerToQuat(euler_rad);

    try {
        auto robot = RobotManager::instance().get(serial_);
        if (!robot) {
            std::cerr << "[MoveRobotAsync] Robot not found: " << serial_ << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        std::array<double, 7> pose;
        pose[0] = target_pos_mm_[0] * MM2M;
        pose[1] = target_pos_mm_[1] * MM2M;
        pose[2] = target_pos_mm_[2] * MM2M;
        pose[3] = target_quat_[0];
        pose[4] = target_quat_[1];
        pose[5] = target_quat_[2];
        pose[6] = target_quat_[3];

        robot->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        robot->SendCartesianMotionForce(
            pose, {}, {},
            getInput<double>("max_linear_vel").value_or(0.5),
            getInput<double>("max_angular_vel").value_or(1.0),
            getInput<double>("max_linear_acc").value_or(2.0),
            getInput<double>("max_angular_acc").value_or(5.0)
        );

        std::cout << "[MoveRobotAsync] Sending motion, waiting for target..." << std::endl;
        return BT::NodeStatus::RUNNING;

    } catch (const std::exception& e) {
        std::cerr << "[MoveRobotAsync] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus MoveRobotAsync::onRunning() {
    try {
        auto robot = RobotManager::instance().get(serial_);
        if (!robot) return BT::NodeStatus::FAILURE;

        auto tcp_pose = robot->states().tcp_pose;

        std::array<double, 3> cur_pos_mm = {
            tcp_pose[0] * M2MM, tcp_pose[1] * M2MM, tcp_pose[2] * M2MM
        };
        std::array<double, 4> cur_quat = {tcp_pose[3], tcp_pose[4], tcp_pose[5], tcp_pose[6]};

        double pos_err = posDist(cur_pos_mm, target_pos_mm_);
        double orient_err = orientDistDeg(cur_quat, target_quat_);

        if (pos_err < pos_tol_mm_ && orient_err < orient_tol_deg_) {
            std::cout << "[MoveRobotAsync] Target reached! pos_err=" << pos_err
                      << "mm orient_err=" << orient_err << "deg" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::RUNNING;

    } catch (const std::exception& e) {
        std::cerr << "[MoveRobotAsync] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

void MoveRobotAsync::onHalted() {
    std::cout << "[MoveRobotAsync] HALTED - stopping robot " << serial_ << std::endl;
    try {
        auto robot = RobotManager::instance().get(serial_);
        if (robot) robot->Stop();
    } catch (...) {}
}

} // namespace flexiv_bt
