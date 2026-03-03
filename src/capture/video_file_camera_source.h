#pragma once

#include "capture/icamera_source.h"
#include <opencv2/videoio.hpp>

namespace mocap {

class VideoFileCameraSource : public ICameraSource {
public:
    VideoFileCameraSource() = default;
    ~VideoFileCameraSource() override;

    bool open(const CameraConfig& config) override;
    void close() override;
    bool isOpened() const override;

    bool grabFrame(CapturedFrame& out, int timeout_ms = 100) override;

    CameraIntrinsics intrinsics() const override;
    std::string id() const override;
    std::string displayName() const override;

    // Video file specific
    int totalFrames() const;
    double fps() const;
    double duration() const;
    bool seek(int frame_number);
    bool seekTime(double seconds);
    int currentFrame() const;

private:
    cv::VideoCapture capture_;
    CameraConfig config_;
    CameraIntrinsics intrinsics_;
    bool intrinsics_loaded_ = false;
    int frame_counter_ = 0;
    int total_frames_ = 0;
    double fps_ = 30.0;
};

}  // namespace mocap
