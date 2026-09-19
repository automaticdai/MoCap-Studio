#include <gtest/gtest.h>
#include "capture/ip_camera_source.h"
#include "pose/onnx_pose_estimator.h"
#include <chrono>
#include <cstdlib>
#include <thread>
#include <cmath>

TEST(PoseModel, CpuInferenceReturnsBody25) {
    const char* model = std::getenv("MOCAP_TEST_MODEL");
    if (!model) GTEST_SKIP() << "Set MOCAP_TEST_MODEL to validate downloaded weights";
    mocap::OnnxPoseEstimator estimator;
    ASSERT_TRUE(estimator.initialize(model, "cpu"));
    estimator.setDetectionThreshold(0.0f);
    const auto poses = estimator.estimate(cv::Mat::zeros(1080, 1920, CV_8UC3));
    ASSERT_EQ(poses.size(), 1u);
    ASSERT_EQ(poses[0].keypoints.size(), 25u);
    for (int i = 0; i < 25; ++i) {
        const auto& kp = poses[0].keypoints[i];
        EXPECT_EQ(kp.index, i);
        EXPECT_TRUE(std::isfinite(kp.x));
        EXPECT_TRUE(std::isfinite(kp.y));
        EXPECT_TRUE(std::isfinite(kp.conf));
    }
    EXPECT_EQ(poses[0].keypoints[24].name, "right_heel");
}

TEST(IpCamera, ClosedSourceReturnsImmediately) {
    mocap::IpCameraSource camera;
    mocap::CapturedFrame frame;
    const auto start = std::chrono::steady_clock::now();
    EXPECT_FALSE(camera.grabFrame(frame, 1000));
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100));
    camera.close();
}

// Opt-in integration test: never opens a network camera in ordinary CI.
TEST(IpCamera, SlowConsumerSkipsFramesAndModelRuns) {
    const char* url = std::getenv("MOCAP_TEST_RTSP_URL");
    if (!url) GTEST_SKIP() << "Set MOCAP_TEST_RTSP_URL for live capture validation";
    mocap::IpCameraSource camera;
    mocap::CameraConfig config;
    config.id = "test-camera";
    config.type = "ip";
    config.url = url;
    ASSERT_TRUE(camera.open(config));
    mocap::CapturedFrame first, latest;
    ASSERT_TRUE(camera.grabFrame(first, 5000));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ASSERT_TRUE(camera.grabFrame(latest, 1000));
    EXPECT_GT(latest.frame_number, first.frame_number + 1);
    EXPECT_GT(latest.timestamp, first.timestamp + 0.5);
    std::cout << "Skipped " << latest.frame_number - first.frame_number - 1
              << " superseded frames for a slow consumer\n";

    const char* model = std::getenv("MOCAP_TEST_MODEL");
    if (model) {
        mocap::OnnxPoseEstimator estimator;
        ASSERT_TRUE(estimator.initialize(model, "cpu"));
        // Validate decoder/layout regardless of whether a person is in view.
        estimator.setDetectionThreshold(0.0f);
        const auto poses = estimator.estimate(latest.image);
        ASSERT_EQ(poses.size(), 1u);
        ASSERT_EQ(poses[0].keypoints.size(), 25u);
        EXPECT_EQ(poses[0].keypoints[1].name, "neck");
        EXPECT_EQ(poses[0].keypoints[8].name, "mid_hip");
        std::cout << "Live inference: 25 BODY_25 joints, confidence "
                  << poses[0].confidence << '\n';
    }
    const auto start = std::chrono::steady_clock::now();
    camera.close();
    EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(2));
    EXPECT_FALSE(camera.isOpened());
    EXPECT_FALSE(camera.grabFrame(latest));
    ASSERT_TRUE(camera.open(config));
    EXPECT_TRUE(camera.grabFrame(latest, 5000));
}
