#include "capture/ip_camera_source.h"
#include <spdlog/spdlog.h>
#include <algorithm>

namespace mocap {

IpCameraSource::~IpCameraSource() {
    close();
}

bool IpCameraSource::open(const CameraConfig& config) {
    close();
    config_ = config;
    intrinsics_loaded_ = false;

    if (config.url.empty()) {
        spdlog::error("IP camera '{}' has no URL configured", config.id);
        return false;
    }

    // Explicit FFmpeg backend: open/read timeouts are open-only properties.
    // Drain continuously below, even while the pipeline is paused or busy.
    if (!capture_.open(config.url, cv::CAP_FFMPEG, {
            cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 5000,
            cv::CAP_PROP_READ_TIMEOUT_MSEC, 1000})) {
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
    running_ = true;
    reader_ = std::thread(&IpCameraSource::readLoop, this);

    spdlog::info("Opened IP camera '{}' at {}", config.id, config.url);
    return true;
}

void IpCameraSource::close() {
    running_ = false;
    frame_ready_.notify_all();
    if (reader_.joinable()) reader_.join();
    if (capture_.isOpened()) {
        capture_.release();
        spdlog::info("Closed IP camera '{}'", config_.id);
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = {};
    frame_available_ = false;
}

bool IpCameraSource::isOpened() const {
    return running_.load();
}

bool IpCameraSource::grabFrame(CapturedFrame& out, int timeout_ms) {
    std::unique_lock<std::mutex> lock(frame_mutex_);
    frame_ready_.wait_for(lock, std::chrono::milliseconds(std::max(0, timeout_ms)),
                         [this] { return frame_available_ || !running_; });
    if (!running_ || !frame_available_) return false;
    out = std::move(latest_frame_);
    frame_available_ = false;
    return true;
}

void IpCameraSource::readLoop() {
    try {
        while (running_) {
            cv::Mat image;
            if (!capture_.read(image) || image.empty()) break;
            CapturedFrame frame;
            frame.camera_id = config_.id;
            frame.timestamp = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start_time_).count();
            frame.frame_number = frame_counter_++;
            frame.image = std::move(image);
            {
                std::lock_guard<std::mutex> lock(frame_mutex_);
                latest_frame_ = std::move(frame); // Overwrite unconsumed frames.
                frame_available_ = true;
            }
            frame_ready_.notify_one();
        }
    } catch (const cv::Exception& e) {
        spdlog::warn("IP camera '{}' read failed: {}", config_.id, e.what());
    }
    running_ = false;
    frame_ready_.notify_all();
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
