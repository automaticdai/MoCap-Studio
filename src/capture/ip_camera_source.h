#pragma once

#include "capture/icamera_source.h"
#include <opencv2/videoio.hpp>
#include <chrono>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace mocap {

class IpCameraSource : public ICameraSource {
public:
    IpCameraSource() = default;
    ~IpCameraSource() override;

    bool open(const CameraConfig& config) override;
    void close() override;
    bool isOpened() const override;

    bool grabFrame(CapturedFrame& out, int timeout_ms = 100) override;
    bool prefersLatestFrame() const override { return true; }

    CameraIntrinsics intrinsics() const override;
    std::string id() const override;
    std::string displayName() const override;

private:
    void readLoop();

    cv::VideoCapture capture_;
    std::thread reader_;
    std::atomic<bool> running_{false};
    std::mutex frame_mutex_;
    std::condition_variable frame_ready_;
    CapturedFrame latest_frame_;
    bool frame_available_ = false;
    CameraConfig config_;
    CameraIntrinsics intrinsics_;
    bool intrinsics_loaded_ = false;
    int frame_counter_ = 0;
    std::chrono::steady_clock::time_point start_time_;
};

}  // namespace mocap
