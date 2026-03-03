#include "capture/video_file_camera_source.h"
#include <spdlog/spdlog.h>

namespace mocap {

VideoFileCameraSource::~VideoFileCameraSource() {
    close();
}

bool VideoFileCameraSource::open(const CameraConfig& config) {
    config_ = config;

    if (config.file_path.empty()) {
        spdlog::error("Video file camera '{}' has no file path configured", config.id);
        return false;
    }

    if (!capture_.open(config.file_path)) {
        spdlog::error("Failed to open video file '{}' for camera '{}'",
                      config.file_path, config.id);
        return false;
    }

    fps_ = capture_.get(cv::CAP_PROP_FPS);
    if (fps_ <= 0.0) fps_ = 30.0;

    total_frames_ = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_COUNT));

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

    spdlog::info("Opened video file '{}' for camera '{}' ({} frames @ {:.1f} fps)",
                 config.file_path, config.id, total_frames_, fps_);
    return true;
}

void VideoFileCameraSource::close() {
    if (capture_.isOpened()) {
        capture_.release();
        spdlog::info("Closed video file camera '{}'", config_.id);
    }
}

bool VideoFileCameraSource::isOpened() const {
    return capture_.isOpened();
}

bool VideoFileCameraSource::grabFrame(CapturedFrame& out, int /*timeout_ms*/) {
    if (!capture_.isOpened()) return false;

    cv::Mat frame;
    if (!capture_.read(frame)) return false;
    if (frame.empty()) return false;

    double timestamp = frame_counter_ / fps_;

    out.camera_id = config_.id;
    out.timestamp = timestamp;
    out.frame_number = frame_counter_++;
    out.image = std::move(frame);

    return true;
}

CameraIntrinsics VideoFileCameraSource::intrinsics() const {
    return intrinsics_;
}

std::string VideoFileCameraSource::id() const {
    return config_.id;
}

std::string VideoFileCameraSource::displayName() const {
    return "Video File (" + config_.id + ") " + config_.file_path;
}

int VideoFileCameraSource::totalFrames() const {
    return total_frames_;
}

double VideoFileCameraSource::fps() const {
    return fps_;
}

double VideoFileCameraSource::duration() const {
    if (fps_ <= 0.0) return 0.0;
    return total_frames_ / fps_;
}

bool VideoFileCameraSource::seek(int frame_number) {
    if (!capture_.isOpened()) return false;
    if (frame_number < 0 || frame_number >= total_frames_) return false;

    capture_.set(cv::CAP_PROP_POS_FRAMES, frame_number);
    frame_counter_ = frame_number;
    return true;
}

bool VideoFileCameraSource::seekTime(double seconds) {
    int frame = static_cast<int>(seconds * fps_);
    return seek(frame);
}

int VideoFileCameraSource::currentFrame() const {
    return frame_counter_;
}

}  // namespace mocap
