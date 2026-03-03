#include <gtest/gtest.h>
#include "triangulation/triangulator.h"
#include <cmath>

using namespace mocap;

namespace {

// Helper to create a simple camera setup
// Camera looking along -Z with given position and focal length
Triangulator::CameraView makeCamera(
    const std::string& id,
    double tx, double ty, double tz,
    double fx = 500.0, int w = 1000, int h = 1000
) {
    Triangulator::CameraView cam;
    cam.camera_id = id;

    cam.intrinsics.fx = fx;
    cam.intrinsics.fy = fx;
    cam.intrinsics.cx = w / 2.0;
    cam.intrinsics.cy = h / 2.0;
    cam.intrinsics.image_size = cv::Size(w, h);
    cam.intrinsics.distortion_coeffs = cv::Mat::zeros(1, 5, CV_64F);

    // Camera at position (tx, ty, tz) looking at origin
    // For simplicity: identity rotation, translation = -position
    cam.extrinsics.rotation = cv::Mat::eye(3, 3, CV_64F);
    cam.extrinsics.translation = cv::Vec3d(-tx, -ty, -tz);

    return cam;
}

// Project a 3D point to 2D using a camera
cv::Point2f project(const Triangulator::CameraView& cam, const Vec3f& pt) {
    cv::Mat P = cam.extrinsics.projectionMatrix(cam.intrinsics);
    cv::Mat X = (cv::Mat_<double>(4, 1) << pt.x(), pt.y(), pt.z(), 1.0);
    cv::Mat proj = P * X;
    double w = proj.at<double>(2);
    return cv::Point2f(
        static_cast<float>(proj.at<double>(0) / w),
        static_cast<float>(proj.at<double>(1) / w)
    );
}

}  // namespace

class TriangulatorTest : public ::testing::Test {
protected:
    Triangulator triangulator;

    void SetUp() override {
        // Set up a two-camera system along the X axis
        auto cam0 = makeCamera("cam_0", -1.0, 0.0, 5.0);
        auto cam1 = makeCamera("cam_1",  1.0, 0.0, 5.0);
        triangulator.setCameras({cam0, cam1});
        triangulator.setMinViews(2);
        triangulator.setRansacEnabled(false);
    }
};

TEST_F(TriangulatorTest, TwoCameraDLT) {
    // Known 3D point
    Vec3f true_point(0.0f, 0.0f, 0.0f);

    auto cam0 = makeCamera("cam_0", -1.0, 0.0, 5.0);
    auto cam1 = makeCamera("cam_1",  1.0, 0.0, 5.0);

    cv::Point2f obs0 = project(cam0, true_point);
    cv::Point2f obs1 = project(cam1, true_point);

    std::vector<std::pair<int, cv::Point2f>> observations = {
        {0, obs0}, {1, obs1}
    };

    auto [point, error] = triangulator.triangulateSinglePoint(observations);

    EXPECT_NEAR(point.x(), 0.0f, 0.1f);
    EXPECT_NEAR(point.y(), 0.0f, 0.1f);
    EXPECT_NEAR(point.z(), 0.0f, 0.1f);
    EXPECT_LT(error, 2.0f);
}

TEST_F(TriangulatorTest, ThreeCameraDLT) {
    auto cam0 = makeCamera("cam_0", -1.0, 0.0, 5.0);
    auto cam1 = makeCamera("cam_1",  1.0, 0.0, 5.0);
    auto cam2 = makeCamera("cam_2",  0.0, 1.0, 5.0);
    triangulator.setCameras({cam0, cam1, cam2});

    Vec3f true_point(0.5f, -0.3f, 0.2f);

    std::vector<std::pair<int, cv::Point2f>> observations = {
        {0, project(cam0, true_point)},
        {1, project(cam1, true_point)},
        {2, project(cam2, true_point)}
    };

    auto [point, error] = triangulator.triangulateSinglePoint(observations);

    EXPECT_NEAR(point.x(), true_point.x(), 0.1f);
    EXPECT_NEAR(point.y(), true_point.y(), 0.1f);
    EXPECT_NEAR(point.z(), true_point.z(), 0.1f);
    EXPECT_LT(error, 2.0f);
}

TEST_F(TriangulatorTest, RANSACRejectsOutlier) {
    auto cam0 = makeCamera("cam_0", -1.0, 0.0, 5.0);
    auto cam1 = makeCamera("cam_1",  1.0, 0.0, 5.0);
    auto cam2 = makeCamera("cam_2",  0.0, 1.0, 5.0);
    auto cam3 = makeCamera("cam_3",  0.0, -1.0, 5.0);
    triangulator.setCameras({cam0, cam1, cam2, cam3});
    triangulator.setRansacEnabled(true);
    triangulator.setRansacThreshold(5.0f);

    Vec3f true_point(0.3f, 0.2f, 0.1f);

    // 3 good observations + 1 outlier
    std::vector<std::pair<int, cv::Point2f>> observations = {
        {0, project(cam0, true_point)},
        {1, project(cam1, true_point)},
        {2, project(cam2, true_point)},
        {3, cv::Point2f(900.0f, 900.0f)}  // outlier
    };

    auto [point, error] = triangulator.triangulateSinglePoint(observations);

    // Should still get a good result despite outlier
    EXPECT_NEAR(point.x(), true_point.x(), 0.2f);
    EXPECT_NEAR(point.y(), true_point.y(), 0.2f);
    EXPECT_NEAR(point.z(), true_point.z(), 0.2f);
}

TEST_F(TriangulatorTest, InsufficientViews) {
    triangulator.setMinViews(2);

    // Only 1 observation
    std::vector<std::pair<int, cv::Point2f>> observations = {
        {0, cv::Point2f(500, 500)}
    };

    auto [point, error] = triangulator.triangulateSinglePoint(observations);

    // Should return zero/high error for insufficient views
    EXPECT_GT(error, 1000.0f);
}

TEST_F(TriangulatorTest, FullTriangulateWithTrackedPersons) {
    auto cam0 = makeCamera("cam_0", -1.0, 0.0, 5.0);
    auto cam1 = makeCamera("cam_1",  1.0, 0.0, 5.0);
    triangulator.setCameras({cam0, cam1});

    Vec3f true_point(0.0f, 0.0f, 0.0f);
    cv::Point2f obs0 = project(cam0, true_point);
    cv::Point2f obs1 = project(cam1, true_point);

    // Create tracked person detections
    PersonTracker::TrackedPerson2D det0;
    det0.global_person_id = 0;
    det0.camera_id = "cam_0";
    det0.pose.person_id = 0;
    det0.pose.confidence = 0.9f;
    {
        Keypoint2D kp;
        kp.index = 0;
        kp.name = "nose";
        kp.x = obs0.x;
        kp.y = obs0.y;
        kp.conf = 0.9f;
        det0.pose.keypoints.push_back(kp);
    }

    PersonTracker::TrackedPerson2D det1;
    det1.global_person_id = 0;
    det1.camera_id = "cam_1";
    det1.pose.person_id = 0;
    det1.pose.confidence = 0.9f;
    {
        Keypoint2D kp;
        kp.index = 0;
        kp.name = "nose";
        kp.x = obs1.x;
        kp.y = obs1.y;
        kp.conf = 0.9f;
        det1.pose.keypoints.push_back(kp);
    }

    auto results = triangulator.triangulate({det0, det1}, 0.0);
    ASSERT_EQ(results.size(), 1u);
    EXPECT_EQ(results[0].global_person_id, 0);
    ASSERT_GE(results[0].markers.size(), 1u);

    EXPECT_NEAR(results[0].markers[0].position.x(), 0.0f, 0.1f);
    EXPECT_NEAR(results[0].markers[0].position.y(), 0.0f, 0.1f);
    EXPECT_NEAR(results[0].markers[0].position.z(), 0.0f, 0.1f);
}
