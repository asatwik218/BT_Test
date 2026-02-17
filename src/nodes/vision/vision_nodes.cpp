#include "nodes/vision/vision_nodes.hpp"
#include "nodes/camera/camera_nodes.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <cmath>
#include <sstream>

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

namespace vision_bt {

constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / M_PI;

// ============================================================================
// Kinematics helpers (Flexiv ZYX convention)
// ============================================================================

/**
 * Build 4x4 homogeneous matrix from [x, y, z, roll, pitch, yaw] using
 * ZYX intrinsic Euler convention (Flexiv).
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)
 */
static Eigen::Matrix4d cartesianPoseToHomMat_ZYX(const std::array<double, 6>& pose) {
    double x = pose[0], y = pose[1], z = pose[2];
    double roll  = pose[3] * DEG_TO_RAD;
    double pitch = pose[4] * DEG_TO_RAD;
    double yaw   = pose[5] * DEG_TO_RAD;

    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ())
      * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H.block<3,3>(0,0) = R;
    H(0,3) = x;
    H(1,3) = y;
    H(2,3) = z;
    return H;
}

static Eigen::Matrix4d translationToHomMat(double dx, double dy, double dz) {
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H(0,3) = dx;
    H(1,3) = dy;
    H(2,3) = dz;
    return H;
}

static Eigen::Matrix4d xRotationToHomMat(double theta_deg) {
    double theta = theta_deg * DEG_TO_RAD;
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H(1,1) =  std::cos(theta);
    H(1,2) = -std::sin(theta);
    H(2,1) =  std::sin(theta);
    H(2,2) =  std::cos(theta);
    return H;
}

static Eigen::Matrix4d yRotationToHomMat(double theta_deg) {
    double theta = theta_deg * DEG_TO_RAD;
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H(0,0) =  std::cos(theta);
    H(0,2) =  std::sin(theta);
    H(2,0) = -std::sin(theta);
    H(2,2) =  std::cos(theta);
    return H;
}

static Eigen::Matrix4d zRotationToHomMat(double theta_deg) {
    double theta = theta_deg * DEG_TO_RAD;
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    H(0,0) =  std::cos(theta);
    H(0,1) = -std::sin(theta);
    H(1,0) =  std::sin(theta);
    H(1,1) =  std::cos(theta);
    return H;
}

static int signOf(double v) {
    if (v > 0.0) return 1;
    if (v < 0.0) return -1;
    return 0;
}

// ============================================================================
// ArUco dictionary name → OpenCV enum mapping
// ============================================================================
static const std::map<std::string, cv::aruco::PredefinedDictionaryType> aruco_dict_map = {
    {"DICT_4X4_50",           cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100",          cv::aruco::DICT_4X4_100},
    {"DICT_4X4_250",          cv::aruco::DICT_4X4_250},
    {"DICT_4X4_1000",         cv::aruco::DICT_4X4_1000},
    {"DICT_5X5_50",           cv::aruco::DICT_5X5_50},
    {"DICT_5X5_100",          cv::aruco::DICT_5X5_100},
    {"DICT_5X5_250",          cv::aruco::DICT_5X5_250},
    {"DICT_5X5_1000",         cv::aruco::DICT_5X5_1000},
    {"DICT_6X6_50",           cv::aruco::DICT_6X6_50},
    {"DICT_6X6_100",          cv::aruco::DICT_6X6_100},
    {"DICT_6X6_250",          cv::aruco::DICT_6X6_250},
    {"DICT_6X6_1000",         cv::aruco::DICT_6X6_1000},
    {"DICT_7X7_50",           cv::aruco::DICT_7X7_50},
    {"DICT_7X7_100",          cv::aruco::DICT_7X7_100},
    {"DICT_7X7_250",          cv::aruco::DICT_7X7_250},
    {"DICT_7X7_1000",         cv::aruco::DICT_7X7_1000},
    {"DICT_ARUCO_ORIGINAL",   cv::aruco::DICT_ARUCO_ORIGINAL},
    {"DICT_APRILTAG_16h5",    cv::aruco::DICT_APRILTAG_16h5},
    {"DICT_APRILTAG_25h9",    cv::aruco::DICT_APRILTAG_25h9},
    {"DICT_APRILTAG_36h10",   cv::aruco::DICT_APRILTAG_36h10},
    {"DICT_APRILTAG_36h11",   cv::aruco::DICT_APRILTAG_36h11},
};

// ============================================================================
// Helper: borrow frame from camera, copy to cv::Mat, release immediately
// ============================================================================
static bool captureFrame(
    const std::string& camera_name,
    const std::string& store_as,
    int& out_width,
    int& out_height
) {
    auto& cam_mgr = camera_bt::CameraManager::instance();
    auto cam = cam_mgr.get(camera_name);
    if (!cam) {
        std::cerr << "[CaptureStereoPair] Camera not found: " << camera_name << std::endl;
        return false;
    }

    std::cout << "[CaptureStereoPair] Borrowing frame from: " << camera_name << std::endl;
    cynlr::camera::FrameBuffer frame{};
    auto err = cam->borrowNewestFrame(frame);
    if (err.has_value()) {
        std::cerr << "[CaptureStereoPair] Failed to borrow frame from: " << camera_name
                  << " error: " << err->message << std::endl;
        return false;
    }

    std::cout << "[CaptureStereoPair] Frame borrowed: " << frame.width << "x" << frame.height
              << " ch=" << frame.channels << " data=" << (frame.data ? "valid" : "NULL") << std::endl;

    if (!frame.data || frame.width <= 0 || frame.height <= 0) {
        std::cerr << "[CaptureStereoPair] Invalid frame data from: " << camera_name << std::endl;
        cam->releaseFrame(frame);
        return false;
    }

    // Convert FrameBuffer to cv::Mat (deep copy)
    // Default to 1 channel if channels field is 0 (Aravis backend may not populate it)
    int ch = (frame.channels > 0) ? frame.channels : 1;
    int cv_type = CV_8UC(ch);
    std::cout << "[CaptureStereoPair] Creating cv::Mat " << frame.width << "x" << frame.height
              << " type=CV_8UC" << ch << std::endl;
    cv::Mat mat(frame.height, frame.width, cv_type, frame.data);
    cv::Mat owned = mat.clone();  // deep copy — safe after release
    std::cout << "[CaptureStereoPair] Mat cloned, releasing frame" << std::endl;

    // Release the borrowed frame immediately
    cam->releaseFrame(frame);

    // Store in VisionManager
    VisionManager::instance().storeFrame(store_as, owned);
    out_width = owned.cols;
    out_height = owned.rows;

    std::cout << "[CaptureStereoPair] Captured " << store_as
              << " (" << out_width << "x" << out_height << ") from " << camera_name << std::endl;
    return true;
}

// ============================================================================
// CaptureStereoPair
// ============================================================================
BT::NodeStatus CaptureStereoPair::tick() {
    auto left_cam = getInput<std::string>("left_camera");
    auto right_cam = getInput<std::string>("right_camera");

    if (!left_cam) {
        std::cerr << "[CaptureStereoPair] Missing input: left_camera" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!right_cam) {
        std::cerr << "[CaptureStereoPair] Missing input: right_camera" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    int lw = 0, lh = 0, rw = 0, rh = 0;

    if (!captureFrame(left_cam.value(), "left", lw, lh)) {
        return BT::NodeStatus::FAILURE;
    }

    if (!captureFrame(right_cam.value(), "right", rw, rh)) {
        // Clean up left frame on failure
        VisionManager::instance().clearFrame("left");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("left_width", lw);
    setOutput("left_height", lh);
    setOutput("right_width", rw);
    setOutput("right_height", rh);

    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// DetectAruco
// ============================================================================
BT::NodeStatus DetectAruco::tick() {
    auto frame_name = getInput<std::string>("frame_name");
    auto dict_name = getInput<std::string>("dictionary");

    if (!frame_name) {
        std::cerr << "[DetectAruco] Missing input: frame_name" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!dict_name) {
        std::cerr << "[DetectAruco] Missing input: dictionary" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Look up dictionary (case-insensitive: convert to uppercase)
    std::string dict_key = dict_name.value();
    std::transform(dict_key.begin(), dict_key.end(), dict_key.begin(), ::toupper);
    auto dict_it = aruco_dict_map.find(dict_key);
    if (dict_it == aruco_dict_map.end()) {
        std::cerr << "[DetectAruco] Unknown dictionary: " << dict_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Get frame from VisionManager
    auto frame_opt = VisionManager::instance().getFrame(frame_name.value());
    if (!frame_opt.has_value()) {
        std::cerr << "[DetectAruco] No frame found in VisionManager: " << frame_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    cv::Mat frame = frame_opt.value();

    // Convert to grayscale if needed (ArUco detection requires single-channel)
    cv::Mat gray;
    if (frame.channels() == 1) {
        gray = frame;
    } else {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    }

    // Create detector and detect markers
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(dict_it->second);
    cv::aruco::DetectorParameters params;
    cv::aruco::ArucoDetector detector(dictionary, params);

    ArucoResult result;
    detector.detectMarkers(gray, result.corners, result.ids, result.rejected);

    int count = static_cast<int>(result.ids.size());
    std::cout << "[DetectAruco] Detected " << count << " markers in '"
              << frame_name.value() << "'" << std::endl;

    // Build comma-separated ID list
    std::ostringstream oss;
    for (size_t i = 0; i < result.ids.size(); ++i) {
        if (i > 0) oss << ",";
        oss << result.ids[i];
    }

    // Store detection results in VisionManager
    VisionManager::instance().storeDetection(frame_name.value(), result);

    setOutput("marker_count", count);
    setOutput("detected_ids", oss.str());

    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ClearVision
// ============================================================================
BT::NodeStatus ClearVision::tick() {
    VisionManager::instance().clearAll();
    std::cout << "[ClearVision] All vision data cleared" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// IncrementsToDegreesMotors
// ============================================================================
BT::NodeStatus IncrementsToDegreesMotors::tick() {
    auto l_incr = getInput<double>("left_motor_increment");
    auto r_incr = getInput<double>("right_motor_increment");
    auto l_par  = getInput<double>("left_parallel_angle");
    auto r_par  = getInput<double>("right_parallel_angle");
    auto ipd    = getInput<double>("increments_per_degree");

    if (!l_incr || !r_incr || !l_par || !r_par || !ipd) {
        std::cerr << "[IncrementsToDegreesMotors] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double incr_per_deg = ipd.value();

    // Left motor: 90 - delta if increment < parallel, else 90 + delta
    double l_delta = std::abs(l_incr.value() - l_par.value()) / incr_per_deg;
    double left_deg = (l_incr.value() < l_par.value()) ? (90.0 - l_delta) : (90.0 + l_delta);

    // Right motor: 90 - delta if increment > parallel, else 90 + delta
    double r_delta = std::abs(r_incr.value() - r_par.value()) / incr_per_deg;
    double right_deg = (r_incr.value() > r_par.value()) ? (90.0 - r_delta) : (90.0 + r_delta);

    setOutput("left_degree_angle", left_deg);
    setOutput("right_degree_angle", right_deg);

    std::cout << "[IncrementsToDegreesMotors] Left: " << left_deg
              << " deg, Right: " << right_deg << " deg" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ConvergenceTriangulation
// ============================================================================
BT::NodeStatus ConvergenceTriangulation::tick() {
    auto theta1_opt = getInput<double>("left_degree_angle");
    auto theta2_opt = getInput<double>("right_degree_angle");
    auto baseline_opt = getInput<double>("baseline");

    if (!theta1_opt || !theta2_opt || !baseline_opt) {
        std::cerr << "[ConvergenceTriangulation] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double p = baseline_opt.value();
    double theta1 = theta1_opt.value() * DEG_TO_RAD;
    double theta2 = theta2_opt.value() * DEG_TO_RAD;

    double sin_sum = std::sin(M_PI - theta1 - theta2);
    if (std::abs(sin_sum) < 1e-12) {
        std::cerr << "[ConvergenceTriangulation] Degenerate geometry: sin(pi - theta1 - theta2) ~ 0" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double k = p / sin_sum;
    double d1 = k * std::sin(theta2);
    double d2 = k * std::sin(theta1);

    double d = std::sqrt((p * p * 0.25) + d2 * d2 - p * d2 * std::cos(theta2));

    double denom = p - 2.0 * d2 * std::cos(theta2);
    double numer = 2.0 * d2 * std::sin(theta2);
    double Theta_deg = std::atan2(numer, denom) * RAD_TO_DEG;

    setOutput("depth", d);
    setOutput("pan_angle_deg", Theta_deg);
    setOutput("d1", d1);
    setOutput("d2", d2);

    std::cout << "[ConvergenceTriangulation] depth=" << d
              << ", pan=" << Theta_deg << " deg, d1=" << d1
              << ", d2=" << d2 << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// PixelOffsetsToMotorIncrements
// ============================================================================
BT::NodeStatus PixelOffsetsToMotorIncrements::tick() {
    auto lx = getInput<double>("left_x_offset");
    auto rx = getInput<double>("right_x_offset");
    auto lm = getInput<double>("left_motor_increment");
    auto rm = getInput<double>("right_motor_increment");
    auto pdr = getInput<double>("pixel_to_degree_ratio");
    auto ipd = getInput<double>("increments_per_degree");

    if (!lx || !rx || !lm || !rm || !pdr || !ipd) {
        std::cerr << "[PixelOffsetsToMotorIncrements] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double ratio = pdr.value();
    double incr = ipd.value();

    double l_target = lm.value() + (lx.value() * ratio * incr);
    double r_target = rm.value() + (rx.value() * ratio * incr);

    setOutput("left_target_increment", l_target);
    setOutput("right_target_increment", r_target);

    std::cout << "[PixelOffsetsToMotorIncrements] Left target: " << l_target
              << ", Right target: " << r_target << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// YPixelOffsetToAngle
// ============================================================================
BT::NodeStatus YPixelOffsetToAngle::tick() {
    auto vfov_opt = getInput<double>("vfov");
    auto yoff_opt = getInput<double>("y_pixel_offset");
    auto half_h_opt = getInput<double>("image_half_height");

    if (!vfov_opt || !yoff_opt || !half_h_opt) {
        std::cerr << "[YPixelOffsetToAngle] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double y_angle = vfov_opt.value() * (yoff_opt.value() / half_h_opt.value());

    setOutput("y_angle_deg", y_angle);

    std::cout << "[YPixelOffsetToAngle] y_angle=" << y_angle << " deg" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ComputeConvergencePoint
// ============================================================================
BT::NodeStatus ComputeConvergencePoint::tick() {
    auto pose_opt   = getInput<std::array<double, 6>>("robot_pose");
    auto depth_opt  = getInput<double>("depth");
    auto pan_opt    = getInput<double>("pan_angle_deg");
    auto yangle_opt = getInput<double>("y_angle_deg");
    auto a6_opt     = getInput<double>("a6_rotation_angle");
    auto hfl_opt    = getInput<double>("height_from_link");
    auto lfa_opt    = getInput<double>("length_from_axis");

    if (!pose_opt || !depth_opt || !pan_opt || !yangle_opt || !a6_opt || !hfl_opt || !lfa_opt) {
        std::cerr << "[ComputeConvergencePoint] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Step 1: Robot pose → homogeneous matrix (Flexiv ZYX convention)
    Eigen::Matrix4d robotPoseMat = cartesianPoseToHomMat_ZYX(pose_opt.value());

    // Step 2: Camera displacement from link
    Eigen::Matrix4d camOffsetMat = translationToHomMat(0.0, hfl_opt.value(), lfa_opt.value());

    // Step 3: A6 (last joint) rotation about Z axis
    Eigen::Matrix4d a6RotMat = zRotationToHomMat(a6_opt.value());

    // Step 4: Pan rotation about Y axis
    // Convert from convergence Theta to Y rotation: (|Theta| - 90) * (-sign(Theta))
    double pan_deg = pan_opt.value();
    double y_rotation_angle = (std::abs(pan_deg) - 90.0) * (-signOf(pan_deg));
    Eigen::Matrix4d yRotMat = yRotationToHomMat(y_rotation_angle);

    // Step 5: Tilt rotation about X axis (from Y pixel offset)
    double y_angle = yangle_opt.value();
    Eigen::Matrix4d xRotMat = xRotationToHomMat(y_angle);

    // Step 6: Depth translation along Z, adjusted for tilt
    double d_horizontal = depth_opt.value();
    double d_resultant = d_horizontal / std::cos(y_angle * DEG_TO_RAD);
    Eigen::Matrix4d depthMat = translationToHomMat(0.0, 0.0, d_resultant);

    // Step 7: Chain all transforms
    Eigen::Matrix4d finalMat = robotPoseMat * camOffsetMat * a6RotMat * yRotMat * xRotMat * depthMat;

    // Extract translation (convergence point)
    double cx = finalMat(0, 3);
    double cy = finalMat(1, 3);
    double cz = finalMat(2, 3);

    setOutput("convergence_x", cx);
    setOutput("convergence_y", cy);
    setOutput("convergence_z", cz);

    std::cout << "[ComputeConvergencePoint] Point: (" << cx << ", " << cy << ", " << cz << ")" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// GetArucoCenterOffset
// ============================================================================
BT::NodeStatus GetArucoCenterOffset::tick() {
    auto frame_name = getInput<std::string>("frame_name");
    auto marker_id  = getInput<int>("marker_id");
    auto img_w      = getInput<int>("image_width");
    auto img_h      = getInput<int>("image_height");

    if (!frame_name || !marker_id || !img_w || !img_h) {
        std::cerr << "[GetArucoCenterOffset] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto det_opt = VisionManager::instance().getDetection(frame_name.value());
    if (!det_opt.has_value()) {
        std::cerr << "[GetArucoCenterOffset] No detection found for: " << frame_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    const auto& det = det_opt.value();
    int target_id = marker_id.value();

    if (det.ids.empty()) {
        std::cerr << "[GetArucoCenterOffset] No markers detected in " << frame_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Find the marker: -1 means use the first detected marker
    int idx = -1;
    if (target_id < 0) {
        idx = 0;
        target_id = det.ids[0];
    } else {
        for (size_t i = 0; i < det.ids.size(); ++i) {
            if (det.ids[i] == target_id) {
                idx = static_cast<int>(i);
                break;
            }
        }
    }

    if (idx < 0) {
        std::cerr << "[GetArucoCenterOffset] Marker ID " << target_id
                  << " not found in " << frame_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Compute center of the 4 corners
    const auto& corners = det.corners[idx];
    float cx = 0.0f, cy = 0.0f;
    for (const auto& pt : corners) {
        cx += pt.x;
        cy += pt.y;
    }
    cx /= static_cast<float>(corners.size());
    cy /= static_cast<float>(corners.size());

    // Offset from image center
    double x_offset = static_cast<double>(cx) - (img_w.value() / 2.0);
    double y_offset = static_cast<double>(cy) - (img_h.value() / 2.0);

    setOutput("x_offset", x_offset);
    setOutput("y_offset", y_offset);

    std::cout << "[GetArucoCenterOffset] " << frame_name.value()
              << " marker " << target_id
              << " offset: (" << x_offset << ", " << y_offset << ")" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// CheckConverged
// ============================================================================
BT::NodeStatus CheckConverged::tick() {
    auto lx = getInput<double>("left_x_offset");
    auto rx = getInput<double>("right_x_offset");
    auto tol = getInput<double>("tolerance");

    if (!lx || !rx || !tol) {
        std::cerr << "[CheckConverged] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double tolerance = tol.value();
    bool converged = (std::abs(lx.value()) < tolerance) && (std::abs(rx.value()) < tolerance);

    if (converged) {
        std::cout << "[CheckConverged] Converged! Left=" << lx.value()
                  << ", Right=" << rx.value() << " (tol=" << tolerance << ")" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    std::cout << "[CheckConverged] Not converged. Left=" << lx.value()
              << ", Right=" << rx.value() << " (tol=" << tolerance << ")" << std::endl;
    return BT::NodeStatus::FAILURE;
}

// ============================================================================
// ComputePickPoses
// ============================================================================
BT::NodeStatus ComputePickPoses::tick() {
    auto cx = getInput<double>("convergence_x");
    auto cy = getInput<double>("convergence_y");
    auto cz = getInput<double>("convergence_z");
    auto l_off = getInput<std::array<double, 3>>("left_offset");
    auto r_off = getInput<std::array<double, 3>>("right_offset");
    auto lift_h = getInput<double>("lift_height");

    if (!cx || !cy || !cz || !l_off || !r_off || !lift_h) {
        std::cerr << "[ComputePickPoses] Missing required input port" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    double x = cx.value(), y = cy.value(), z = cz.value();
    auto lo = l_off.value();
    auto ro = r_off.value();
    double lh = lift_h.value();

    std::array<double, 3> left_pick  = {x + lo[0], y + lo[1], z + lo[2]};
    std::array<double, 3> right_pick = {x + ro[0], y + ro[1], z + ro[2]};
    std::array<double, 3> lift_left  = {left_pick[0],  left_pick[1],  left_pick[2] + lh};
    std::array<double, 3> lift_right = {right_pick[0], right_pick[1], right_pick[2] + lh};

    setOutput("left_pick_position", left_pick);
    setOutput("right_pick_position", right_pick);
    setOutput("lift_left_position", lift_left);
    setOutput("lift_right_position", lift_right);

    std::cout << "[ComputePickPoses] Left pick: (" << left_pick[0] << ", " << left_pick[1] << ", " << left_pick[2] << ")"
              << " Right pick: (" << right_pick[0] << ", " << right_pick[1] << ", " << right_pick[2] << ")" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SaveFrame
// ============================================================================
BT::NodeStatus SaveFrame::tick() {
    auto frame_name = getInput<std::string>("frame_name");
    auto file_path = getInput<std::string>("file_path");

    if (!frame_name) {
        std::cerr << "[SaveFrame] Missing input: frame_name" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!file_path) {
        std::cerr << "[SaveFrame] Missing input: file_path" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto frame_opt = VisionManager::instance().getFrame(frame_name.value());
    if (!frame_opt) {
        std::cerr << "[SaveFrame] Frame not found: " << frame_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    bool ok = cv::imwrite(file_path.value(), frame_opt.value());
    if (!ok) {
        std::cerr << "[SaveFrame] Failed to write: " << file_path.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SaveFrame] Saved " << frame_name.value() << " ("
              << frame_opt.value().cols << "x" << frame_opt.value().rows
              << ") to " << file_path.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace vision_bt
