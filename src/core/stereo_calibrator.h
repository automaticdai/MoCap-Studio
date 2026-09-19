#pragma once

#include "core/intrinsics_calibrator.h"
#include <string>

namespace mocap {

struct StereoResult {
    // World-to-camera transform for camera 2; camera 1 defines the world frame.
    CameraExtrinsics extrinsics;
    double rms = 0.0;
    bool success = false;
    std::string error;
};

// Each index must describe the same stationary board pose in both cameras.
// Translation uses metres, as specified by CheckerboardSpec::square_size_m.
StereoResult calibrateStereo(
    const std::vector<std::vector<cv::Point2f>>& reference_views,
    const std::vector<std::vector<cv::Point2f>>& other_views,
    const CameraIntrinsics& reference,
    const CameraIntrinsics& other,
    const CheckerboardSpec& board);

}  // namespace mocap
