#include "capture/ip_camera_source.h"
#include <spdlog/spdlog.h>

namespace mocap {

IpCameraSource::~IpCameraSource() {
    close();
}

bool IpCameraSource::open(const CameraConfig& config) {
    config_ = config;

    if (config.url.empty()) {
        spdlog::error("IP camera '{}' has no URL configured", config.id);
        return false;
    }

    if (!capture_.open(config.url)) {
        spdlog::error("Failed to open IP camera '{}' at URL: {}", config.id, config.url);
        return false;
    }

    if (!config.intrinsics_file.empty()) {
        try {
            intrinsics_ = CameraIntrinsics::loadFromYaml(config.intrinsics_file);
            intrinsics_loaded_ = true;
        } catch (const std::exception& e) {
            spdlog::warn("Failed to load intrinsics for camera {}: {}", config.id, e.what());
        }
    }

    if (!intrinsics_loaded_) {
        intrinsics_.image_size = cv::Size(
            static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT))
        );
        intrinsics_.cx = intrinsics_.image_size.width / 2.0;
        intrinsics_.cy = intrinsics_.image_size.height / 2.0;
        intrinsics_.fx = intrinsics_.image_size.width;
        intrinsics_.fy = intrinsics_.image_size.width;
    }

    frame_counter_ = 0;
    start_time_ = std::chrono::steady_clock::now();

    spdlog::info("Opened IP camera '{}' at {}", config.id, config.url);
    return true;
}

void IpCameraSource::close() {
    if (capture_.isOpened()) {
        capture_.release();
        spdlog::info("Closed IP camera '{}'", config_.id);
    }
}

bool IpCameraSource::isOpened() const {
    return capture_.isOpened();
}

bool IpCameraSource::grabFrame(CapturedFrame& out, int /*timeout_ms*/) {
    if (!capture_.isOpened()) return false;

    cv::Mat frame;
    if (!capture_.read(frame)) return false;
    if (frame.empty()) return false;

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - start_time_).count();

    out.camera_id = config_.id;
    out.timestamp = elapsed;
    out.frame_number = frame_counter_++;
    out.image = std::move(frame);

    return true;
}

CameraIntrinsics IpCameraSource::intrinsics() const {
    return intrinsics_;
}

std::string IpCameraSource::id() const {
    return config_.id;
}

std::string IpCameraSource::displayName() const {
    return "IP Camera (" + config_.id + ") " + config_.url;
}

}  // namespace mocap
