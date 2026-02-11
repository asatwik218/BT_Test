#include "flexiv_nodes.hpp"
#include "flexiv/rdk/mode.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>
#include <thread>
#include <chrono>

#ifndef M_PI
    #define M_PI 3.14
#endif
namespace flexiv_bt {

// ============================================================================
// Constants
// ============================================================================

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double MM_TO_M = 0.001;
constexpr double M_TO_MM = 1000.0;

// ============================================================================
// Helper functions
// ============================================================================

/**
 * @brief Convert Euler ZYX angles (in radians) to quaternion
 * @param euler_rad [roll (x), pitch (y), yaw (z)] in radians
 * @return Quaternion as [qw, qx, qy, qz]
 */
static std::array<double, 4> eulerZYXToQuat(const std::array<double, 3>& euler_rad) {
    Eigen::AngleAxisd rollAngle(euler_rad[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler_rad[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler_rad[2], Eigen::Vector3d::UnitZ());

    // Compose rotations in ZYX order: R = yaw * pitch * roll
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

    return {q.w(), q.x(), q.y(), q.z()};
}

/**
 * @brief Convert quaternion to Euler ZYX angles (in radians)
 * @param quat Quaternion as [qw, qx, qy, qz]
 * @return Euler angles [roll (x), pitch (y), yaw (z)] in radians
 */
static std::array<double, 3> quatToEulerZYX(const std::array<double, 4>& quat) {
    Eigen::Quaterniond q(quat[0], quat[1], quat[2], quat[3]);
    // Get Euler angles in ZYX order (returns [roll, pitch, yaw])
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX order
    // eulerAngles returns [yaw, pitch, roll], we need [roll, pitch, yaw]
    return {euler[2], euler[1], euler[0]};
}

/**
 * @brief Compute Euclidean distance between two 3D positions
 */
static double positionDistance(const std::array<double, 3>& a, const std::array<double, 3>& b) {
    double dx = a[0] - b[0];
    double dy = a[1] - b[1];
    double dz = a[2] - b[2];
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/**
 * @brief Compute quaternion distance (1 - |q1·q2|)
 * Returns 0 for identical orientations, up to 1 for 180° difference
 */
static double quaternionDistance(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
    // q1 and q2 are [qw, qx, qy, qz]
    double dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
    return 1.0 - std::abs(dot);
}

/**
 * @brief Compute angular distance between two orientations in degrees
 */
static double orientationDistanceDeg(const std::array<double, 4>& q1, const std::array<double, 4>& q2) {
    double dot = std::abs(q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]);
    dot = std::min(1.0, dot);  // Clamp to avoid numerical issues with acos
    double angle_rad = 2.0 * std::acos(dot);
    return angle_rad * RAD_TO_DEG;
}

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
// MoveRobot Implementation
// ============================================================================

BT::NodeStatus MoveRobot::tick() {
    // Get required inputs
    auto serial = getInput<std::string>("serial");
    auto position_mm = getInput<std::array<double, 3>>("position_mm");
    auto orientation_deg = getInput<std::array<double, 3>>("orientation_deg");

    if (!serial) {
        std::cerr << "[MoveRobot] Missing 'serial': " << serial.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!position_mm) {
        std::cerr << "[MoveRobot] Missing 'position_mm': " << position_mm.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!orientation_deg) {
        std::cerr << "[MoveRobot] Missing 'orientation_deg': " << orientation_deg.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Get optional inputs with defaults
    double max_linear_vel = getInput<double>("max_linear_vel").value_or(0.5);
    double max_angular_vel = getInput<double>("max_angular_vel").value_or(1.0);
    double max_linear_acc = getInput<double>("max_linear_acc").value_or(2.0);
    double max_angular_acc = getInput<double>("max_angular_acc").value_or(5.0);

    try {
        auto robot = RobotManager::instance().get(serial.value());
        if (!robot) {
            std::cerr << "[MoveRobot] Robot not found: " << serial.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Convert position from mm to meters
        std::array<double, 3> pos_m = {
            position_mm.value()[0] * MM_TO_M,
            position_mm.value()[1] * MM_TO_M,
            position_mm.value()[2] * MM_TO_M
        };

        // Convert orientation from degrees to radians, then to quaternion
        std::array<double, 3> euler_rad = {
            orientation_deg.value()[0] * DEG_TO_RAD,  // roll
            orientation_deg.value()[1] * DEG_TO_RAD,  // pitch
            orientation_deg.value()[2] * DEG_TO_RAD   // yaw
        };
        std::array<double, 4> quat = eulerZYXToQuat(euler_rad);

        // Construct pose array [x, y, z, qw, qx, qy, qz]
        std::array<double, 7> pose;
        pose[0] = pos_m[0];  // x
        pose[1] = pos_m[1];  // y
        pose[2] = pos_m[2];  // z
        pose[3] = quat[0];   // qw
        pose[4] = quat[1];   // qx
        pose[5] = quat[2];   // qy
        pose[6] = quat[3];   // qz

        // Ensure robot is in correct mode
        robot->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);

        std::cout << "[MoveRobot] Sending motion to pos_mm=["
                  << position_mm.value()[0] << ", " << position_mm.value()[1] << ", " << position_mm.value()[2]
                  << "] orient_deg=[" << orientation_deg.value()[0] << ", " << orientation_deg.value()[1]
                  << ", " << orientation_deg.value()[2] << "]" << std::endl;

        // Send async motion command
        robot->SendCartesianMotionForce(
            pose,
            {},  // wrench (empty = no force control)
            {},  // velocity (empty = position control)
            max_linear_vel,
            max_angular_vel,
            max_linear_acc,
            max_angular_acc
        );

        return BT::NodeStatus::SUCCESS;

    } catch (const std::exception& e) {
        std::cerr << "[MoveRobot] Exception: " << e.what() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}

// ============================================================================
// CheckTargetReached Implementation
// ============================================================================

BT::NodeStatus CheckTargetReached::tick() {
    // Get required inputs
    auto serial = getInput<std::string>("serial");
    auto target_position_mm = getInput<std::array<double, 3>>("target_position_mm");
    auto target_orientation_deg = getInput<std::array<double, 3>>("target_orientation_deg");

    if (!serial) {
        std::cerr << "[CheckTargetReached] Missing 'serial': " << serial.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!target_position_mm) {
        std::cerr << "[CheckTargetReached] Missing 'target_position_mm': " << target_position_mm.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!target_orientation_deg) {
        std::cerr << "[CheckTargetReached] Missing 'target_orientation_deg': " << target_orientation_deg.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Get optional inputs with defaults (in mm and degrees)
    double pos_tol_mm = getInput<double>("position_tolerance_mm").value_or(1.0);
    double orient_tol_deg = getInput<double>("orientation_tolerance_deg").value_or(1.0);

    try {
        auto robot = RobotManager::instance().get(serial.value());
        if (!robot) {
            std::cerr << "[CheckTargetReached] Robot not found: " << serial.value() << std::endl;
            return BT::NodeStatus::FAILURE;
        }

        // Get current robot state
        auto states = robot->states();
        auto tcp_pose = states.tcp_pose;  // [x, y, z, qw, qx, qy, qz]

        // Extract current position in mm
        std::array<double, 3> current_pos_mm = {
            tcp_pose[0] * M_TO_MM,
            tcp_pose[1] * M_TO_MM,
            tcp_pose[2] * M_TO_MM
        };

        // Extract current orientation as quaternion
        std::array<double, 4> current_quat = {tcp_pose[3], tcp_pose[4], tcp_pose[5], tcp_pose[6]};

        // Convert target orientation from degrees to quaternion for comparison
        std::array<double, 3> target_euler_rad = {
            target_orientation_deg.value()[0] * DEG_TO_RAD,
            target_orientation_deg.value()[1] * DEG_TO_RAD,
            target_orientation_deg.value()[2] * DEG_TO_RAD
        };
        std::array<double, 4> target_quat = eulerZYXToQuat(target_euler_rad);

        // Compute position distance in mm
        double pos_dist_mm = positionDistance(current_pos_mm, target_position_mm.value());

        // Compute orientation distance in degrees
        double orient_dist_deg = orientationDistanceDeg(current_quat, target_quat);

        // Check if within tolerances
        bool position_reached = pos_dist_mm < pos_tol_mm;
        bool orientation_reached = orient_dist_deg < orient_tol_deg;

        if (position_reached && orientation_reached) {
            std::cout << "[CheckTargetReached] Target reached! pos_err_mm=" << pos_dist_mm
                      << " orient_err_deg=" << orient_dist_deg << std::endl;
            return BT::NodeStatus::SUCCESS;
        } else {
            // Not reached yet
            return BT::NodeStatus::FAILURE;
        }

    } catch (const std::exception& e) {
        std::cerr << "[CheckTargetReached] Exception: " << e.what() << std::endl;
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

} // namespace flexiv_bt
