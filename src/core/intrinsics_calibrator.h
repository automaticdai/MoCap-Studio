#pragma once

#include "core/camera_intrinsics.h"
#include <opencv2/core.hpp>
#include <vector>

namespace mocap {

// Specification of the physical calibration board.
struct CheckerboardSpec {
    int cols = 9;                 // inner corners per row
    int rows = 6;                 // inner corners per column
    double square_size_m = 0.025; // metres per square
};

// Build the 3D object-point grid for a checkerboard, expressed in board
// coordinates (Z=0). Shared by every view of the same board.
std::vector<cv::Point3f> makeBoardObjectPoints(const CheckerboardSpec& spec);

// Run `cv::findChessboardCorners` with sub-pixel refinement on a single image.
// Returns true and fills `out_corners` if the full board is found.
bool detectCheckerboard(const cv::Mat& image_bgr,
                        const CheckerboardSpec& spec,
                        std::vector<cv::Point2f>& out_corners);

struct IntrinsicsResult {
    CameraIntrinsics intrinsics;
    double rms = 0.0;                // overall RMS reprojection error (px)
    std::vector<double> per_view_errors; // mean error per input view (px)
    bool success = false;
};

// Run `cv::calibrateCamera` over the given views and compute per-view mean
// reprojection error in pixels. Returns `success = false` if fewer than 3
// views are supplied or the underlying call fails.
IntrinsicsResult calibrateIntrinsics(
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    const CheckerboardSpec& spec);

}  // namespace mocap
