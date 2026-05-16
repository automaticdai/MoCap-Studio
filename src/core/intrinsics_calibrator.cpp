#include "core/intrinsics_calibrator.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <spdlog/spdlog.h>
#include <cmath>

namespace mocap {

std::vector<cv::Point3f> makeBoardObjectPoints(const CheckerboardSpec& spec) {
    std::vector<cv::Point3f> pts;
    pts.reserve(spec.cols * spec.rows);
    for (int r = 0; r < spec.rows; ++r) {
        for (int c = 0; c < spec.cols; ++c) {
            pts.emplace_back(c * spec.square_size_m, r * spec.square_size_m, 0.0f);
        }
    }
    return pts;
}

bool detectCheckerboard(const cv::Mat& image_bgr,
                        const CheckerboardSpec& spec,
                        std::vector<cv::Point2f>& out_corners) {
    if (image_bgr.empty()) return false;

    cv::Mat gray;
    if (image_bgr.channels() == 1) {
        gray = image_bgr;
    } else {
        cv::cvtColor(image_bgr, gray, cv::COLOR_BGR2GRAY);
    }

    const cv::Size pattern(spec.cols, spec.rows);
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH
                    | cv::CALIB_CB_NORMALIZE_IMAGE
                    | cv::CALIB_CB_FAST_CHECK;

    out_corners.clear();
    if (!cv::findChessboardCorners(gray, pattern, out_corners, flags)) {
        return false;
    }

    cv::cornerSubPix(
        gray, out_corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
    return true;
}

IntrinsicsResult calibrateIntrinsics(
    const std::vector<std::vector<cv::Point2f>>& image_points,
    const cv::Size& image_size,
    const CheckerboardSpec& spec)
{
    IntrinsicsResult result;
    if (image_points.size() < 3) {
        spdlog::warn("calibrateIntrinsics: need ≥3 views, got {}", image_points.size());
        return result;
    }

    const auto object_points_one = makeBoardObjectPoints(spec);
    std::vector<std::vector<cv::Point3f>> object_points(image_points.size(), object_points_one);

    cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    try {
        result.rms = cv::calibrateCamera(
            object_points, image_points, image_size, K, dist, rvecs, tvecs);
    } catch (const cv::Exception& e) {
        spdlog::error("cv::calibrateCamera failed: {}", e.what());
        return result;
    }

    result.intrinsics.fx = K.at<double>(0, 0);
    result.intrinsics.fy = K.at<double>(1, 1);
    result.intrinsics.cx = K.at<double>(0, 2);
    result.intrinsics.cy = K.at<double>(1, 2);
    result.intrinsics.distortion_coeffs = dist.clone();
    result.intrinsics.image_size = image_size;

    // Per-view mean reprojection error: re-project the object points using each
    // view's recovered pose and average the per-corner pixel distance.
    result.per_view_errors.reserve(image_points.size());
    for (size_t i = 0; i < image_points.size(); ++i) {
        std::vector<cv::Point2f> reproj;
        cv::projectPoints(object_points[i], rvecs[i], tvecs[i], K, dist, reproj);

        double sum = 0.0;
        for (size_t j = 0; j < reproj.size(); ++j) {
            const double dx = reproj[j].x - image_points[i][j].x;
            const double dy = reproj[j].y - image_points[i][j].y;
            sum += std::sqrt(dx * dx + dy * dy);
        }
        result.per_view_errors.push_back(sum / static_cast<double>(reproj.size()));
    }

    result.success = true;
    return result;
}

}  // namespace mocap
