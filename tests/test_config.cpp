#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "core/config.h"

using namespace mocap;

class ConfigTest : public ::testing::Test {
protected:
    std::string test_config_path;

    void SetUp() override {
        test_config_path = std::filesystem::temp_directory_path() / "test_config.yaml";

        std::ofstream f(test_config_path);
        f << R"(
capture:
  target_fps: 30
  sync_mode: "hardware"
  max_sync_skew_ms: 3.0

cameras:
  - id: "cam_0"
    type: "usb"
    device_index: 0
    resolution: [1280, 720]
    intrinsics_file: "cal/cam0.yaml"
  - id: "cam_1"
    type: "ip"
    url: "rtsp://192.168.1.100/stream"
    resolution: [1920, 1080]

pose_estimation:
  backend: "onnxruntime"
  model: "rtmpose_l.onnx"
  device: "cpu"
  detection_threshold: 0.6
  keypoint_threshold: 0.4

triangulation:
  min_views: 3
  ransac_enabled: false
  ransac_threshold_px: 8.0
  temporal_filter: "savitzky_golay"
  filter_cutoff_hz: 10.0

skeleton:
  definition: "custom_skeleton"
  ik_solver: "optimisation"
  joint_limits_enabled: false

gui:
  canvas_fps: 30
  default_render_layers: ["grid", "skeleton"]
  colour_palette: "viridis"
)";
    }

    void TearDown() override {
        std::filesystem::remove(test_config_path);
    }
};

TEST_F(ConfigTest, LoadCaptureSettings) {
    auto config = AppConfig::load(test_config_path);
    EXPECT_EQ(config.capture.target_fps, 30);
    EXPECT_EQ(config.capture.sync_mode, "hardware");
    EXPECT_DOUBLE_EQ(config.capture.max_sync_skew_ms, 3.0);
}

TEST_F(ConfigTest, LoadCameras) {
    auto config = AppConfig::load(test_config_path);
    ASSERT_EQ(config.cameras.size(), 2u);

    EXPECT_EQ(config.cameras[0].id, "cam_0");
    EXPECT_EQ(config.cameras[0].type, "usb");
    EXPECT_EQ(config.cameras[0].device_index, 0);
    EXPECT_EQ(config.cameras[0].resolution_width, 1280);
    EXPECT_EQ(config.cameras[0].resolution_height, 720);
    EXPECT_EQ(config.cameras[0].intrinsics_file, "cal/cam0.yaml");

    EXPECT_EQ(config.cameras[1].id, "cam_1");
    EXPECT_EQ(config.cameras[1].type, "ip");
    EXPECT_EQ(config.cameras[1].url, "rtsp://192.168.1.100/stream");
}

TEST_F(ConfigTest, LoadPoseEstimation) {
    auto config = AppConfig::load(test_config_path);
    EXPECT_EQ(config.pose_estimation.backend, "onnxruntime");
    EXPECT_EQ(config.pose_estimation.model, "rtmpose_l.onnx");
    EXPECT_EQ(config.pose_estimation.device, "cpu");
    EXPECT_FLOAT_EQ(config.pose_estimation.detection_threshold, 0.6f);
    EXPECT_FLOAT_EQ(config.pose_estimation.keypoint_threshold, 0.4f);
}

TEST_F(ConfigTest, LoadTriangulation) {
    auto config = AppConfig::load(test_config_path);
    EXPECT_EQ(config.triangulation.min_views, 3);
    EXPECT_FALSE(config.triangulation.ransac_enabled);
    EXPECT_FLOAT_EQ(config.triangulation.ransac_threshold_px, 8.0f);
    EXPECT_EQ(config.triangulation.temporal_filter, "savitzky_golay");
    EXPECT_FLOAT_EQ(config.triangulation.filter_cutoff_hz, 10.0f);
}

TEST_F(ConfigTest, LoadSkeleton) {
    auto config = AppConfig::load(test_config_path);
    EXPECT_EQ(config.skeleton.definition, "custom_skeleton");
    EXPECT_EQ(config.skeleton.ik_solver, "optimisation");
    EXPECT_FALSE(config.skeleton.joint_limits_enabled);
}

TEST_F(ConfigTest, LoadGui) {
    auto config = AppConfig::load(test_config_path);
    EXPECT_EQ(config.gui.canvas_fps, 30);
    ASSERT_EQ(config.gui.default_render_layers.size(), 2u);
    EXPECT_EQ(config.gui.default_render_layers[0], "grid");
    EXPECT_EQ(config.gui.default_render_layers[1], "skeleton");
    EXPECT_EQ(config.gui.colour_palette, "viridis");
}

TEST_F(ConfigTest, DefaultValues) {
    // Write a minimal config with no keys
    std::string minimal_path = std::filesystem::temp_directory_path() / "minimal_config.yaml";
    {
        std::ofstream f(minimal_path);
        f << "---\n";
    }

    auto config = AppConfig::load(minimal_path);
    EXPECT_EQ(config.capture.target_fps, 60);
    EXPECT_EQ(config.capture.sync_mode, "software");
    EXPECT_TRUE(config.cameras.empty());
    EXPECT_EQ(config.pose_estimation.backend, "onnxruntime");
    EXPECT_EQ(config.triangulation.min_views, 2);
    EXPECT_TRUE(config.skeleton.joint_limits_enabled);
    EXPECT_EQ(config.gui.canvas_fps, 60);

    std::filesystem::remove(minimal_path);
}

TEST_F(ConfigTest, SaveAndReload) {
    auto config = AppConfig::load(test_config_path);

    std::string save_path = std::filesystem::temp_directory_path() / "saved_config.yaml";
    config.save(save_path);

    auto reloaded = AppConfig::load(save_path);
    EXPECT_EQ(reloaded.capture.target_fps, config.capture.target_fps);
    EXPECT_EQ(reloaded.capture.sync_mode, config.capture.sync_mode);
    EXPECT_EQ(reloaded.cameras.size(), config.cameras.size());
    EXPECT_EQ(reloaded.pose_estimation.device, config.pose_estimation.device);
    EXPECT_EQ(reloaded.skeleton.definition, config.skeleton.definition);
    EXPECT_EQ(reloaded.gui.colour_palette, config.gui.colour_palette);

    std::filesystem::remove(save_path);
}
