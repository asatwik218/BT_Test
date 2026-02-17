#include "camera_nodes.hpp"

#include <iostream>
#include <unordered_map>

namespace camera_bt {

// ============================================================================
// Pixel format string-to-enum mapping
// ============================================================================
static const std::unordered_map<std::string, cynlr::camera::PixelFormat> pixel_format_map = {
    {"MONO8",  cynlr::camera::PixelFormat::MONO8},
    {"MONO10", cynlr::camera::PixelFormat::MONO10},
    {"MONO12", cynlr::camera::PixelFormat::MONO12},
    {"MONO14", cynlr::camera::PixelFormat::MONO14},
    {"MONO16", cynlr::camera::PixelFormat::MONO16},
};

// ============================================================================
// ConnectCamera Implementation
// ============================================================================

BT::NodeStatus ConnectCamera::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[ConnectCamera] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    int buffer_count = getInput<int>("buffer_count").value_or(10);

    auto cam = CameraManager::instance().create(camera_name.value(), static_cast<uint32_t>(buffer_count));
    if (!cam) {
        std::cerr << "[ConnectCamera] Failed to create camera: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ConnectCamera] Camera connected: " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// DisconnectCamera Implementation
// ============================================================================

BT::NodeStatus DisconnectCamera::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[DisconnectCamera] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Release any borrowed frame before disconnecting
    auto cam = CameraManager::instance().get(camera_name.value());
    if (cam && CameraManager::instance().hasFrame(camera_name.value())) {
        auto frame_opt = CameraManager::instance().getFrame(camera_name.value());
        if (frame_opt) {
            auto frame = frame_opt.value();
            cam->releaseFrame(frame);
        }
    }

    CameraManager::instance().remove(camera_name.value());
    std::cout << "[DisconnectCamera] Camera disconnected: " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StartAcquisition Implementation
// ============================================================================

BT::NodeStatus StartAcquisition::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[StartAcquisition] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[StartAcquisition] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->startAcquisition();
    if (err.has_value()) {
        std::cerr << "[StartAcquisition] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StartAcquisition] Acquisition started: " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// StopAcquisition Implementation
// ============================================================================

BT::NodeStatus StopAcquisition::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[StopAcquisition] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[StopAcquisition] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->stopAcquisition();
    if (err.has_value()) {
        std::cerr << "[StopAcquisition] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[StopAcquisition] Acquisition stopped: " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetExposure Implementation
// ============================================================================

BT::NodeStatus SetExposure::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    auto exposure_us = getInput<double>("exposure_us");

    if (!camera_name) {
        std::cerr << "[SetExposure] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!exposure_us) {
        std::cerr << "[SetExposure] Missing 'exposure_us': " << exposure_us.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetExposure] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setExposureTime(exposure_us.value());
    if (err.has_value()) {
        std::cerr << "[SetExposure] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetExposure] Exposure set to " << exposure_us.value() << " us" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetGain Implementation
// ============================================================================

BT::NodeStatus SetGain::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    auto gain = getInput<double>("gain");

    if (!camera_name) {
        std::cerr << "[SetGain] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!gain) {
        std::cerr << "[SetGain] Missing 'gain': " << gain.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetGain] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setGain(gain.value());
    if (err.has_value()) {
        std::cerr << "[SetGain] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetGain] Gain set to " << gain.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetFrameRate Implementation
// ============================================================================

BT::NodeStatus SetFrameRate::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    auto frame_rate = getInput<double>("frame_rate");

    if (!camera_name) {
        std::cerr << "[SetFrameRate] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!frame_rate) {
        std::cerr << "[SetFrameRate] Missing 'frame_rate': " << frame_rate.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetFrameRate] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setFrameRate(frame_rate.value());
    if (err.has_value()) {
        std::cerr << "[SetFrameRate] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetFrameRate] Frame rate set to " << frame_rate.value() << " fps" << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetPixelFormat Implementation
// ============================================================================

BT::NodeStatus SetPixelFormat::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    auto pixel_format = getInput<std::string>("pixel_format");

    if (!camera_name) {
        std::cerr << "[SetPixelFormat] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!pixel_format) {
        std::cerr << "[SetPixelFormat] Missing 'pixel_format': " << pixel_format.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetPixelFormat] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto it = pixel_format_map.find(pixel_format.value());
    if (it == pixel_format_map.end()) {
        std::cerr << "[SetPixelFormat] Unknown pixel format: " << pixel_format.value()
                  << ". Valid: MONO8, MONO10, MONO12, MONO14, MONO16" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setPixelFormat(it->second);
    if (err.has_value()) {
        std::cerr << "[SetPixelFormat] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetPixelFormat] Pixel format set to " << pixel_format.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetBinning Implementation
// ============================================================================

BT::NodeStatus SetBinning::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[SetBinning] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    int binning_x = getInput<int>("binning_x").value_or(1);
    int binning_y = getInput<int>("binning_y").value_or(1);

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetBinning] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setBinning(binning_x, binning_y);
    if (err.has_value()) {
        std::cerr << "[SetBinning] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetBinning] Binning set to " << binning_x << "x" << binning_y << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetAutoExposure Implementation
// ============================================================================

BT::NodeStatus SetAutoExposure::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    auto enabled = getInput<bool>("enabled");

    if (!camera_name) {
        std::cerr << "[SetAutoExposure] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!enabled) {
        std::cerr << "[SetAutoExposure] Missing 'enabled': " << enabled.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetAutoExposure] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setAutoExposure(enabled.value());
    if (err.has_value()) {
        std::cerr << "[SetAutoExposure] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetAutoExposure] Auto exposure " << (enabled.value() ? "enabled" : "disabled") << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// AcquireFrame Implementation
// ============================================================================

BT::NodeStatus AcquireFrame::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[AcquireFrame] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[AcquireFrame] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Check if a frame is already borrowed (must release first)
    if (CameraManager::instance().hasFrame(camera_name.value())) {
        std::cerr << "[AcquireFrame] Frame already borrowed for: " << camera_name.value()
                  << ". Call ReleaseFrame first." << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    cynlr::camera::FrameBuffer frame{};
    auto err = cam->borrowNewestFrame(frame);
    if (err.has_value()) {
        std::cerr << "[AcquireFrame] Error borrowing frame: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    // Store in manager for zero-copy downstream access
    CameraManager::instance().storeFrame(camera_name.value(), frame);

    // Expose metadata through output ports
    setOutput("width", frame.width);
    setOutput("height", frame.height);
    setOutput("channels", frame.channels);

    std::cout << "[AcquireFrame] Frame acquired: " << frame.width << "x" << frame.height
              << " ch=" << frame.channels << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// ReleaseFrame Implementation
// ============================================================================

BT::NodeStatus ReleaseFrame::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[ReleaseFrame] Missing 'camera_name': " << camera_name.error() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[ReleaseFrame] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto frame_opt = CameraManager::instance().getFrame(camera_name.value());
    if (!frame_opt) {
        std::cerr << "[ReleaseFrame] No frame borrowed for: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto frame = frame_opt.value();
    cam->releaseFrame(frame);
    CameraManager::instance().clearFrame(camera_name.value());

    std::cout << "[ReleaseFrame] Frame released: " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// EnableLensPower Implementation
// ============================================================================

BT::NodeStatus EnableLensPower::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[EnableLensPower] Missing 'camera_name'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    bool enable = getInput<bool>("enable").value_or(true);

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[EnableLensPower] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->enableLensPower(enable);
    if (err.has_value()) {
        std::cerr << "[EnableLensPower] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[EnableLensPower] 3.3V " << (enable ? "enabled" : "disabled")
              << " on " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetLensFocus Implementation
// ============================================================================

BT::NodeStatus SetLensFocus::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    auto voltage = getInput<double>("voltage");

    if (!camera_name) {
        std::cerr << "[SetLensFocus] Missing 'camera_name'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    if (!voltage) {
        std::cerr << "[SetLensFocus] Missing 'voltage'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetLensFocus] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setLensFocus(voltage.value());
    if (err.has_value()) {
        std::cerr << "[SetLensFocus] Error: " << err->message << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::cout << "[SetLensFocus] Focus set to " << voltage.value() << "V on "
              << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ============================================================================
// SetupLensSerial Implementation
// ============================================================================

BT::NodeStatus SetupLensSerial::tick() {
    auto camera_name = getInput<std::string>("camera_name");
    if (!camera_name) {
        std::cerr << "[SetupLensSerial] Missing 'camera_name'" << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    std::string line = getInput<std::string>("line").value_or("Line1");
    std::string source = getInput<std::string>("source").value_or("SerialPort0_Tx");
    std::string baud = getInput<std::string>("baud_rate").value_or("Baud57600");

    auto cam = CameraManager::instance().get(camera_name.value());
    if (!cam) {
        std::cerr << "[SetupLensSerial] Camera not found: " << camera_name.value() << std::endl;
        return BT::NodeStatus::FAILURE;
    }

    auto err = cam->setupLensSerial(line.c_str(), source.c_str(), baud.c_str());
    if (err.has_value()) {
        std::cerr << "[SetupLensSerial] Warning: " << err->message
                  << " (non-fatal, focus may still work)" << std::endl;
    }

    std::cout << "[SetupLensSerial] Serial configured on " << camera_name.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

} // namespace camera_bt
