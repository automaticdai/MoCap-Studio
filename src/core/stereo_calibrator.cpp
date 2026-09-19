#include "core/stereo_calibrator.h"
#include <opencv2/calib3d.hpp>
#include <cmath>

namespace mocap {
StereoResult calibrateStereo(
    const std::vector<std::vector<cv::Point2f>>& reference_views,
    const std::vector<std::vector<cv::Point2f>>& other_views,
    const CameraIntrinsics& reference, const CameraIntrinsics& other,
    const CheckerboardSpec& board) {
    StereoResult result;
    auto fail = [&](const std::string& message) {
        result.error = message;
        return result;
    };
    if (reference_views.size() < 8 || reference_views.size() != other_views.size())
        return fail("At least 8 paired board views are required.");
    if (board.cols < 3 || board.rows < 3 || !std::isfinite(board.square_size_m) ||
        board.square_size_m <= 0)
        return fail("Invalid checkerboard dimensions or square size.");
    for (const auto* intr : {&reference, &other}) {
        if (intr->image_size.width <= 0 || intr->image_size.height <= 0 ||
            !std::isfinite(intr->fx) || !std::isfinite(intr->fy) ||
            !std::isfinite(intr->cx) || !std::isfinite(intr->cy) ||
            intr->fx <= 0 || intr->fy <= 0 ||
            (!intr->distortion_coeffs.empty() && !cv::checkRange(intr->distortion_coeffs)))
            return fail("Valid intrinsics are required for both cameras.");
    }
    const auto points = makeBoardObjectPoints(board);
    for (const auto* views : {&reference_views, &other_views}) {
        double max_motion = 0.0;
        for (const auto& view : *views) {
            if (view.size() != points.size()) return fail("Incomplete paired board corners.");
            for (size_t i = 0; i < view.size(); ++i) {
                if (!std::isfinite(view[i].x) || !std::isfinite(view[i].y))
                    return fail("Non-finite board corners.");
                max_motion = std::max(max_motion, cv::norm(view[i] - views->front()[i]));
            }
        }
        if (max_motion < 5.0) return fail("Move and tilt the board between captures.");
    }
    try {
        cv::Mat k1 = reference.cameraMatrix(), k2 = other.cameraMatrix();
        cv::Mat d1 = reference.distortion_coeffs.clone(), d2 = other.distortion_coeffs.clone();
        cv::Mat rotation, translation, essential, fundamental;
        std::vector<std::vector<cv::Point3f>> objects(reference_views.size(), points);
        result.rms = cv::stereoCalibrate(objects, reference_views, other_views,
            k1, d1, k2, d2, reference.image_size, rotation, translation,
            essential, fundamental, cv::CALIB_FIX_INTRINSIC,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-7));
        if (!std::isfinite(result.rms) || !cv::checkRange(rotation) || !cv::checkRange(translation))
            return fail("Stereo calibration produced invalid geometry.");
        if (result.rms > 2.0)
            return fail("Stereo error exceeds 2 px. Hold the board still for each capture, keep its orientation consistent, and recapture varied views.");
        if (cv::norm(translation) < 0.001)
            return fail("Camera separation is too small. Check the selected streams and square size.");
        result.extrinsics.rotation = rotation.clone();
        result.extrinsics.translation = cv::Vec3d(translation.at<double>(0),
            translation.at<double>(1), translation.at<double>(2));
        result.success = true;
    } catch (const cv::Exception& e) {
        return fail(std::string("Stereo calibration failed: ") + e.what());
    }
    return result;
}
}  // namespace mocap
