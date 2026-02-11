#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "Camera.hpp"
#include "AravisBackend.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <iostream>

namespace camera_bt {

/**
 * @brief Singleton manager for Camera instances and borrowed frames.
 * Cameras keyed by name string. Also stores borrowed FrameBuffers
 * for zero-copy downstream access.
 */
class CameraManager {
public:
    static CameraManager& instance() {
        static CameraManager inst;
        return inst;
    }

    /**
     * @brief Create and store a camera by name
     * @param name Camera identifier (passed to AravisBackend::create)
     * @param buffer_count Number of stream buffers
     * @return Shared pointer to Camera, or nullptr on failure
     */
    std::shared_ptr<cynlr::camera::Camera> create(
        const std::string& name,
        uint32_t buffer_count = 10
    ) {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = cameras_.find(name);
        if (it != cameras_.end()) {
            std::cout << "[CameraManager] Camera already exists: " << name << std::endl;
            return it->second;
        }

        std::cout << "[CameraManager] Creating camera: " << name << std::endl;
        auto backend = cynlr::camera::AravisBackend::create(name.c_str(), buffer_count);
        if (!backend) {
            std::cerr << "[CameraManager] Failed to create AravisBackend for: " << name << std::endl;
            return nullptr;
        }

        auto cam = std::make_shared<cynlr::camera::Camera>(std::move(backend));
        cameras_[name] = cam;
        std::cout << "[CameraManager] Camera created successfully: " << name << std::endl;
        return cam;
    }

    std::shared_ptr<cynlr::camera::Camera> get(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = cameras_.find(name);
        return (it != cameras_.end()) ? it->second : nullptr;
    }

    void remove(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        // Release any borrowed frame before removing
        frames_.erase(name);
        cameras_.erase(name);
        std::cout << "[CameraManager] Camera removed: " << name << std::endl;
    }

    bool exists(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return cameras_.find(name) != cameras_.end();
    }

    // --- Frame management (zero-copy) ---

    void storeFrame(const std::string& name, const cynlr::camera::FrameBuffer& frame) {
        std::lock_guard<std::mutex> lock(mutex_);
        frames_[name] = frame;
    }

    std::optional<cynlr::camera::FrameBuffer> getFrame(const std::string& name) const {
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

    bool hasFrame(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return frames_.find(name) != frames_.end();
    }

private:
    CameraManager() = default;
    ~CameraManager() = default;
    CameraManager(const CameraManager&) = delete;
    CameraManager& operator=(const CameraManager&) = delete;

    std::map<std::string, std::shared_ptr<cynlr::camera::Camera>> cameras_;
    std::map<std::string, cynlr::camera::FrameBuffer> frames_;
    mutable std::mutex mutex_;
};

// ============================================================================
// Node 1: ConnectCamera
// ============================================================================
/**
 * @brief Creates camera backend and stores in CameraManager
 *
 * Ports:
 *   - camera_name [input]: Unique camera identifier
 *   - buffer_count [input]: Number of stream buffers (default 10)
 *
 * Returns SUCCESS if created, FAILURE on error
 */
class ConnectCamera : public BT::SyncActionNode {
public:
    ConnectCamera(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<int>("buffer_count", 10, "Number of stream buffers")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 2: DisconnectCamera
// ============================================================================
class DisconnectCamera : public BT::SyncActionNode {
public:
    DisconnectCamera(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("camera_name", "Camera identifier") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 3: StartAcquisition
// ============================================================================
class StartAcquisition : public BT::SyncActionNode {
public:
    StartAcquisition(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("camera_name", "Camera identifier") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 4: StopAcquisition
// ============================================================================
class StopAcquisition : public BT::SyncActionNode {
public:
    StopAcquisition(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("camera_name", "Camera identifier") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 5: SetExposure
// ============================================================================
class SetExposure : public BT::SyncActionNode {
public:
    SetExposure(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<double>("exposure_us", "Exposure time in microseconds")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 6: SetGain
// ============================================================================
class SetGain : public BT::SyncActionNode {
public:
    SetGain(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<double>("gain", "Gain value")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 7: SetFrameRate
// ============================================================================
class SetFrameRate : public BT::SyncActionNode {
public:
    SetFrameRate(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<double>("frame_rate", "Frame rate in fps")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 8: SetPixelFormat
// ============================================================================
class SetPixelFormat : public BT::SyncActionNode {
public:
    SetPixelFormat(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<std::string>("pixel_format", "Pixel format: MONO8, MONO10, MONO12, MONO14, MONO16")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 9: SetBinning
// ============================================================================
class SetBinning : public BT::SyncActionNode {
public:
    SetBinning(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<int>("binning_x", 1, "Horizontal binning"),
            BT::InputPort<int>("binning_y", 1, "Vertical binning")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 10: SetAutoExposure
// ============================================================================
class SetAutoExposure : public BT::SyncActionNode {
public:
    SetAutoExposure(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::InputPort<bool>("enabled", "Enable or disable auto exposure")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 11: AcquireFrame
// ============================================================================
/**
 * @brief Borrows the newest frame and stores it in CameraManager.
 * Downstream nodes access frame data via CameraManager::getFrame().
 *
 * Ports:
 *   - camera_name [input]: Camera identifier
 *   - width [output]: Frame width in pixels
 *   - height [output]: Frame height in pixels
 *   - channels [output]: Number of channels
 */
class AcquireFrame : public BT::SyncActionNode {
public:
    AcquireFrame(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("camera_name", "Camera identifier"),
            BT::OutputPort<int>("width", "Frame width"),
            BT::OutputPort<int>("height", "Frame height"),
            BT::OutputPort<int>("channels", "Frame channels")
        };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Node 12: ReleaseFrame
// ============================================================================
/**
 * @brief Releases a previously borrowed frame back to the stream.
 * Must be called before the next AcquireFrame.
 */
class ReleaseFrame : public BT::SyncActionNode {
public:
    ReleaseFrame(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}

    static BT::PortsList providedPorts() {
        return { BT::InputPort<std::string>("camera_name", "Camera identifier") };
    }

    BT::NodeStatus tick() override;
};

// ============================================================================
// Registration helper
// ============================================================================
inline void registerCameraNodes(BT::BehaviorTreeFactory& factory) {
    factory.registerNodeType<ConnectCamera>("ConnectCamera");
    factory.registerNodeType<DisconnectCamera>("DisconnectCamera");
    factory.registerNodeType<StartAcquisition>("StartAcquisition");
    factory.registerNodeType<StopAcquisition>("StopAcquisition");
    factory.registerNodeType<SetExposure>("SetExposure");
    factory.registerNodeType<SetGain>("SetGain");
    factory.registerNodeType<SetFrameRate>("SetFrameRate");
    factory.registerNodeType<SetPixelFormat>("SetPixelFormat");
    factory.registerNodeType<SetBinning>("SetBinning");
    factory.registerNodeType<SetAutoExposure>("SetAutoExposure");
    factory.registerNodeType<AcquireFrame>("AcquireFrame");
    factory.registerNodeType<ReleaseFrame>("ReleaseFrame");
}

} // namespace camera_bt
