#include "capture/usb_camera_source.h"
#include <spdlog/spdlog.h>

namespace mocap {

UsbCameraSource::~UsbCameraSource() {
    close();
}

bool UsbCameraSource::open(const CameraConfig& config) {
    config_ = config;

    if (!capture_.open(config.device_index)) {
        spdlog::error("Failed to open USB camera at index {}", config.device_index);
        return false;
    }

    capture_.set(cv::CAP_PROP_FRAME_WIDTH, config.resolution_width);
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, config.resolution_height);

    if (!config.intrinsics_file.empty()) {
        try {
            intrinsics_ = CameraIntrinsics::loadFromYaml(config.intrinsics_file);
            intrinsics_loaded_ = true;
        } catch (const std::exception& e) {
            spdlog::warn("Failed to load intrinsics for camera {}: {}", config.id, e.what());
        }
    }

    if (!intrinsics_loaded_) {
        // Set basic intrinsics from capture properties
        intrinsics_.image_size = cv::Size(
            static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH)),
            static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT))
        );
        intrinsics_.cx = intrinsics_.image_size.width / 2.0;
        intrinsics_.cy = intrinsics_.image_size.height / 2.0;
        intrinsics_.fx = intrinsics_.image_size.width;  // rough estimate
        intrinsics_.fy = intrinsics_.image_size.width;
    }

    frame_counter_ = 0;
    start_time_ = std::chrono::steady_clock::now();

    spdlog::info("Opened USB camera '{}' at index {} ({}x{})",
                 config.id, config.device_index,
                 intrinsics_.image_size.width, intrinsics_.image_size.height);
    return true;
}

void UsbCameraSource::close() {
    if (capture_.isOpened()) {
        capture_.release();
        spdlog::info("Closed USB camera '{}'", config_.id);
    }
}

bool UsbCameraSource::isOpened() const {
    return capture_.isOpened();
}

bool UsbCameraSource::grabFrame(CapturedFrame& out, int /*timeout_ms*/) {
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

CameraIntrinsics UsbCameraSource::intrinsics() const {
    return intrinsics_;
}

std::string UsbCameraSource::id() const {
    return config_.id;
}

std::string UsbCameraSource::displayName() const {
    return "USB Camera " + std::to_string(config_.device_index) + " (" + config_.id + ")";
}

}  // namespace mocap
