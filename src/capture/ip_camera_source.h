#pragma once

#include "capture/icamera_source.h"
#include <opencv2/videoio.hpp>
#include <chrono>

namespace mocap {

class IpCameraSource : public ICameraSource {
public:
    IpCameraSource() = default;
    ~IpCameraSource() override;

    bool open(const CameraConfig& config) override;
    void close() override;
    bool isOpened() const override;

    bool grabFrame(CapturedFrame& out, int timeout_ms = 100) override;

    CameraIntrinsics intrinsics() const override;
    std::string id() const override;
    std::string displayName() const override;

private:
    cv::VideoCapture capture_;
    CameraConfig config_;
    CameraIntrinsics intrinsics_;
    bool intrinsics_loaded_ = false;
    int frame_counter_ = 0;
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace mocap
