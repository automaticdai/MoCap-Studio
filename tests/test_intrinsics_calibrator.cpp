// Synthetic-data test for the intrinsics calibrator: project a known board
// through a known camera at many poses, then recover the camera parameters
// and verify they are close to ground truth.

#include "core/intrinsics_calibrator.h"
#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <random>

using namespace mocap;

namespace {

// Project board points through a camera at the given pose and return the
// resulting pixel coordinates. No noise — these are perfect observations.
std::vector<cv::Point2f> projectBoardAt(
    const std::vector<cv::Point3f>& object_points,
    const cv::Vec3d& rvec,
    const cv::Vec3d& tvec,
    const cv::Mat& K,
    const cv::Mat& dist)
{
    std::vector<cv::Point2f> image_points;
    cv::projectPoints(object_points, rvec, tvec, K, dist, image_points);
    return image_points;
}

}  // namespace

TEST(IntrinsicsCalibratorTest, BoardObjectPointsHaveExpectedShape) {
    CheckerboardSpec spec;
    spec.cols = 9;
    spec.rows = 6;
    spec.square_size_m = 0.025;

    auto pts = makeBoardObjectPoints(spec);
    ASSERT_EQ(pts.size(), 9u * 6u);
    EXPECT_FLOAT_EQ(pts.front().x, 0.0f);
    EXPECT_FLOAT_EQ(pts.front().y, 0.0f);
    // Last corner is at ((cols-1)*square, (rows-1)*square, 0).
    EXPECT_FLOAT_EQ(pts.back().x, 8 * 0.025f);
    EXPECT_FLOAT_EQ(pts.back().y, 5 * 0.025f);
    EXPECT_FLOAT_EQ(pts.back().z, 0.0f);
}

TEST(IntrinsicsCalibratorTest, RecoversIntrinsicsFromSyntheticViews) {
    // Ground-truth camera (1080p, mild pinhole, no distortion).
    const cv::Size image_size(1920, 1080);
    cv::Mat K_true = (cv::Mat_<double>(3, 3) <<
        1500.0,    0.0, 960.0,
           0.0, 1500.0, 540.0,
           0.0,    0.0,   1.0);
    cv::Mat dist_true = cv::Mat::zeros(1, 5, CV_64F);

    CheckerboardSpec spec;
    spec.cols = 9;
    spec.rows = 6;
    spec.square_size_m = 0.025;
    auto object_points = makeBoardObjectPoints(spec);

    // Generate 15 views with varied board poses. Translation pushes the board
    // ~1m in front of the camera; rotations stay small so the board remains
    // fully visible.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> rot(-0.3, 0.3);     // radians
    std::uniform_real_distribution<double> tx(-0.15, 0.15);
    std::uniform_real_distribution<double> tz(0.6, 1.4);

    std::vector<std::vector<cv::Point2f>> image_points;
    for (int v = 0; v < 15; ++v) {
        cv::Vec3d rvec(rot(rng), rot(rng), rot(rng));
        cv::Vec3d tvec(tx(rng), tx(rng), tz(rng));
        image_points.push_back(projectBoardAt(object_points, rvec, tvec, K_true, dist_true));
    }

    auto result = calibrateIntrinsics(image_points, image_size, spec);
    ASSERT_TRUE(result.success);

    // Noise-free synthetic data — recovery should be tight. Tolerances picked
    // to be comfortably above OpenCV's optimisation residual but tight enough
    // to flag a real regression.
    EXPECT_NEAR(result.intrinsics.fx, 1500.0, 5.0);
    EXPECT_NEAR(result.intrinsics.fy, 1500.0, 5.0);
    EXPECT_NEAR(result.intrinsics.cx, 960.0, 5.0);
    EXPECT_NEAR(result.intrinsics.cy, 540.0, 5.0);
    EXPECT_LT(result.rms, 0.5);

    ASSERT_EQ(result.per_view_errors.size(), image_points.size());
    for (double err : result.per_view_errors) {
        EXPECT_LT(err, 1.0) << "per-view error should be sub-pixel on noise-free data";
    }
}

TEST(IntrinsicsCalibratorTest, RejectsTooFewViews) {
    std::vector<std::vector<cv::Point2f>> two_views(2);
    auto result = calibrateIntrinsics(two_views, cv::Size(1920, 1080), {});
    EXPECT_FALSE(result.success);
}
