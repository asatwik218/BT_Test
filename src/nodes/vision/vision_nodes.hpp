#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "Frame.hpp"

#include <opencv2/core.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>

#include <array>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

// ============================================================================
// Type converter for std::array<double, 6> (robot pose: "x;y;z;roll;pitch;yaw")
// ============================================================================
namespace BT {

template <>
inline std::array<double, 6> convertFromString(StringView str) {
    std::array<double, 6> result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::string input(str.data(), str.size());
    std::istringstream iss(input);
    std::string token;
    size_t index = 0;
    while (std::getline(iss, token, ';') && index < 6) {
        result[index++] = std::stod(token);
    }
    if (index != 6) {
        throw RuntimeError("Cannot convert '", str,
            "' to std::array<double, 6>. Expected format: 'x;y;z;roll;pitch;yaw'");
    }
    return result;
}

} // namespace BT

namespace vision_bt {

// ============================================================================
// ArUco detection result
// ============================================================================
struct ArucoResult {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<std::vector<cv::Point2f>> rejected;
};

// ============================================================================
// VisionManager singleton — stores cv::Mat frames and detection results
// ============================================================================
class VisionManager {
public:
    static VisionManager& instance() {
        static VisionManager inst;
        return inst;
    }

    // --- Frame storage (deep-copied cv::Mat) ---

    void storeFrame(const std::string& name, const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_[name] = frame.clone();
    }

    std::optional<cv::Mat> getFrame(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = frames_.find(name);
        if (it != frames_.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    void clearFrame(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.erase(name);
    }

    // --- ArUco detection results ---

    void storeDetection(const std::string& name, const ArucoResult& result) {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_[name] = result;
    }

    std::optional<ArucoResult> getDetection(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = detections_.find(name);
        if (it != detections_.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    void clearDetection(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        detections_.erase(name);
    }

    void clearAll() {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_.clear();
        detections_.clear();
    }

private:
    VisionManager() = default;
    ~VisionManager() = default;
    VisionManager(const VisionManager&) = delete;
    VisionManager& operator=(const VisionManager&) = delete;

    std::map<std::string, cv::Mat> frames_;
    std::map<std::string, ArucoResult> detections_;
    mutable std::mutex mutex_;
};

// ============================================================================
// Node 1: CaptureStereoPair
// ============================================================================
/**
 * Borrows newest frame from left and right cameras (via CameraManager),
 * deep-copies to cv::Mat, releases frames immediately, stores cv::Mat
 * in VisionManager as "left" and "right".
 *
 * Ports:
 *   - left_camera [input]: Name of left camera in CameraManager
 *   - right_camera [input]: Name of right camera in CameraManager
 *   - left_width [output]: Left frame width
 *   - left_height [output]: Left frame height
 *   - right_width [output]: Right frame width
 *   - right_height [output]: Right frame height
 */
class CaptureStereoPair : public BT::SyncActionNode {
public:
    CaptureStereoPair(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("left_camera", "Left camera name in CameraManager"),
            BT::InputPort<std::string>("right_camera", "Right camera name in CameraManager"),
            BT::OutputPort<int>("left_width", "Left frame width"),
            BT::OutputPort<int>("left_height", "Left frame height"),
            BT::OutputPort<int>("right_width", "Right frame width"),
            BT::OutputPort<int>("right_height", "Right frame height")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 2: DetectAruco
// ============================================================================
/**
 * Detects ArUco markers in a frame stored in VisionManager.
 * Stores detection results (ids + corners) in VisionManager.
 *
 * Ports:
 *   - frame_name [input]: Key in VisionManager ("left" or "right")
 *   - dictionary [input]: ArUco dictionary name (e.g. "DICT_4X4_50")
 *   - marker_count [output]: Number of detected markers
 *   - detected_ids [output]: Comma-separated list of detected marker IDs
 */
class DetectAruco : public BT::SyncActionNode {
public:
    DetectAruco(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("frame_name", "Frame key in VisionManager"),
            BT::InputPort<std::string>("dictionary", "DICT_4X4_50",
                "ArUco dictionary: DICT_4X4_50, DICT_4X4_100, DICT_4X4_250, DICT_4X4_1000, "
                "DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000, "
                "DICT_6X6_50, DICT_6X6_100, DICT_6X6_250, DICT_6X6_1000, "
                "DICT_7X7_50, DICT_7X7_100, DICT_7X7_250, DICT_7X7_1000, "
                "DICT_ARUCO_ORIGINAL, DICT_APRILTAG_16h5, DICT_APRILTAG_25h9, "
                "DICT_APRILTAG_36h10, DICT_APRILTAG_36h11"),
            BT::OutputPort<int>("marker_count", "Number of detected markers"),
            BT::OutputPort<std::string>("detected_ids", "Comma-separated detected marker IDs")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 3: ClearVision
// ============================================================================
/**
 * Clears all frames and detection results from VisionManager.
 * Call this at the end of a vision processing sequence to free memory.
 */
class ClearVision : public BT::SyncActionNode {
public:
    ClearVision(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {};
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 4: IncrementsToDegreesMotors
// ============================================================================
/**
 * Converts left/right motor increments + parallel angles to degree angles.
 *
 * Left motor:  90 - |increment - parallel| / incr_per_deg  if increment < parallel
 *              90 + |increment - parallel| / incr_per_deg  otherwise
 * Right motor: 90 - |increment - parallel| / incr_per_deg  if increment > parallel
 *              90 + |increment - parallel| / incr_per_deg  otherwise
 */
class IncrementsToDegreesMotors : public BT::SyncActionNode {
public:
    IncrementsToDegreesMotors(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("left_motor_increment", "Left motor raw increment"),
            BT::InputPort<double>("right_motor_increment", "Right motor raw increment"),
            BT::InputPort<double>("left_parallel_angle", "Left motor parallel reference angle"),
            BT::InputPort<double>("right_parallel_angle", "Right motor parallel reference angle"),
            BT::InputPort<double>("increments_per_degree", 364.08, "Motor increments per degree"),
            BT::OutputPort<double>("left_degree_angle", "Left motor angle in degrees"),
            BT::OutputPort<double>("right_degree_angle", "Right motor angle in degrees")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 5: ConvergenceTriangulation
// ============================================================================
/**
 * Computes depth and pan angle from stereo degree angles using law of sines.
 * Given theta1 (left) and theta2 (right), with baseline p:
 *   k = p / sin(pi - theta1 - theta2)
 *   d1 = k * sin(theta2),  d2 = k * sin(theta1)
 *   d = sqrt(p^2/4 + d2^2 - p*d2*cos(theta2))
 *   Theta = atan((2*d2*sin(theta2)) / (p - 2*d2*cos(theta2)))
 */
class ConvergenceTriangulation : public BT::SyncActionNode {
public:
    ConvergenceTriangulation(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("left_degree_angle", "Left motor angle in degrees"),
            BT::InputPort<double>("right_degree_angle", "Right motor angle in degrees"),
            BT::InputPort<double>("baseline", 90.0, "Baseline distance (mm)"),
            BT::OutputPort<double>("depth", "Horizontal depth d"),
            BT::OutputPort<double>("pan_angle_deg", "Convergence pan angle Theta (degrees)"),
            BT::OutputPort<double>("d1", "Distance d1"),
            BT::OutputPort<double>("d2", "Distance d2")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 6: PixelOffsetsToMotorIncrements
// ============================================================================
/**
 * Converts pixel X offsets to target motor increments (for centering loop).
 * target = current_increment + x_offset * pixel_to_degree_ratio * increments_per_degree
 */
class PixelOffsetsToMotorIncrements : public BT::SyncActionNode {
public:
    PixelOffsetsToMotorIncrements(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("left_x_offset", "Left image X pixel offset"),
            BT::InputPort<double>("right_x_offset", "Right image X pixel offset"),
            BT::InputPort<double>("left_motor_increment", "Current left motor increment"),
            BT::InputPort<double>("right_motor_increment", "Current right motor increment"),
            BT::InputPort<double>("pixel_to_degree_ratio", 0.04902, "Pixel-to-degree ratio (15/306)"),
            BT::InputPort<double>("increments_per_degree", 364.0, "Motor increments per degree"),
            BT::OutputPort<double>("left_target_increment", "Target left motor increment"),
            BT::OutputPort<double>("right_target_increment", "Target right motor increment")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 7: YPixelOffsetToAngle
// ============================================================================
/**
 * Converts vertical pixel offset to degree angle.
 * y_angle_deg = vfov * (y_pixel_offset / image_half_height)
 */
class YPixelOffsetToAngle : public BT::SyncActionNode {
public:
    YPixelOffsetToAngle(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("vfov", "Vertical field of view (degrees)"),
            BT::InputPort<double>("y_pixel_offset", "Y pixel offset from center"),
            BT::InputPort<double>("image_half_height", 256.0, "Half of image height in pixels"),
            BT::OutputPort<double>("y_angle_deg", "Vertical angle in degrees")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 8: ComputeConvergencePoint
// ============================================================================
/**
 * Chains homogeneous transforms to compute the 3D convergence point:
 *   Robot_Pose (ZYX) -> Camera_Offset -> A6_Rotation(Z) -> Pan(Y) -> Tilt(X) -> Depth(Z)
 *
 * Uses Flexiv ZYX intrinsic Euler convention for the robot pose.
 */
class ComputeConvergencePoint : public BT::SyncActionNode {
public:
    ComputeConvergencePoint(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::array<double, 6>>("robot_pose",
                "Robot Cartesian pose [x;y;z;roll;pitch;yaw] in mm/degrees"),
            BT::InputPort<double>("depth", "Horizontal depth from triangulation (mm)"),
            BT::InputPort<double>("pan_angle_deg", "Pan angle from triangulation (degrees)"),
            BT::InputPort<double>("y_angle_deg", "Tilt angle from Y pixel offset (degrees)"),
            BT::InputPort<double>("a6_rotation_angle", "Last joint rotation angle (degrees)"),
            BT::InputPort<double>("height_from_link", -3.0, "Camera height offset from link (mm)"),
            BT::InputPort<double>("length_from_axis", 84.9, "Camera length offset from axis (mm)"),
            BT::OutputPort<double>("convergence_x", "Convergence point X (mm)"),
            BT::OutputPort<double>("convergence_y", "Convergence point Y (mm)"),
            BT::OutputPort<double>("convergence_z", "Convergence point Z (mm)")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 9: GetArucoCenterOffset
// ============================================================================
/**
 * Reads ArUco detection from VisionManager for the given frame,
 * finds the specified marker_id, computes center of its 4 corners,
 * and outputs pixel offset from image center.
 */
class GetArucoCenterOffset : public BT::SyncActionNode {
public:
    GetArucoCenterOffset(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("frame_name", "VisionManager key (\"left\" or \"right\")"),
            BT::InputPort<int>("marker_id", 0, "ArUco marker ID to look for"),
            BT::InputPort<int>("image_width", "Frame width in pixels"),
            BT::InputPort<int>("image_height", "Frame height in pixels"),
            BT::OutputPort<double>("x_offset", "Horizontal pixel offset from center"),
            BT::OutputPort<double>("y_offset", "Vertical pixel offset from center")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 10: CheckConverged
// ============================================================================
/**
 * Condition node: returns SUCCESS if both left and right X offsets
 * are within the specified pixel tolerance, FAILURE otherwise.
 */
class CheckConverged : public BT::SyncActionNode {
public:
    CheckConverged(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("left_x_offset", "Left image X pixel offset"),
            BT::InputPort<double>("right_x_offset", "Right image X pixel offset"),
            BT::InputPort<double>("tolerance", 5.0, "Pixel tolerance for convergence")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 11: ComputePickPoses
// ============================================================================
/**
 * Takes convergence point (individual doubles) and per-arm offsets,
 * produces std::array<double,3> positions for MoveRobot.
 * Also computes lift positions (pick position + lift_height along Z).
 */
class ComputePickPoses : public BT::SyncActionNode {
public:
    ComputePickPoses(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("convergence_x", "Convergence point X (mm)"),
            BT::InputPort<double>("convergence_y", "Convergence point Y (mm)"),
            BT::InputPort<double>("convergence_z", "Convergence point Z (mm)"),
            BT::InputPort<std::array<double, 3>>("left_offset", "0;-50;0",
                "Left arm offset from convergence [dx;dy;dz] (mm)"),
            BT::InputPort<std::array<double, 3>>("right_offset", "0;50;0",
                "Right arm offset from convergence [dx;dy;dz] (mm)"),
            BT::InputPort<double>("lift_height", 50.0, "Lift height above pick position (mm)"),
            BT::OutputPort<std::array<double, 3>>("left_pick_position", "Left arm pick position [x;y;z]"),
            BT::OutputPort<std::array<double, 3>>("right_pick_position", "Right arm pick position [x;y;z]"),
            BT::OutputPort<std::array<double, 3>>("lift_left_position", "Left arm lift position [x;y;z]"),
            BT::OutputPort<std::array<double, 3>>("lift_right_position", "Right arm lift position [x;y;z]")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 12: SaveFrame
// ============================================================================
/**
 * Saves a frame from VisionManager to disk as an image file (PNG/JPG/BMP).
 * Useful for debugging and verifying camera capture.
 */
class SaveFrame : public BT::SyncActionNode {
public:
    SaveFrame(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("frame_name", "Frame key in VisionManager (e.g. \"left\")"),
            BT::InputPort<std::string>("file_path", "Output file path (e.g. \"captured.png\")")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Registration helper
// ============================================================================
inline void registerVisionNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<CaptureStereoPair>("CaptureStereoPair");
    factory.registerNodeType<DetectAruco>("DetectAruco");
    factory.registerNodeType<ClearVision>("ClearVision");
    factory.registerNodeType<IncrementsToDegreesMotors>("IncrementsToDegreesMotors");
    factory.registerNodeType<ConvergenceTriangulation>("ConvergenceTriangulation");
    factory.registerNodeType<PixelOffsetsToMotorIncrements>("PixelOffsetsToMotorIncrements");
    factory.registerNodeType<YPixelOffsetToAngle>("YPixelOffsetToAngle");
    factory.registerNodeType<ComputeConvergencePoint>("ComputeConvergencePoint");
    factory.registerNodeType<GetArucoCenterOffset>("GetArucoCenterOffset");
    factory.registerNodeType<CheckConverged>("CheckConverged");
    factory.registerNodeType<ComputePickPoses>("ComputePickPoses");
    factory.registerNodeType<SaveFrame>("SaveFrame");
}

} // namespace vision_bt
