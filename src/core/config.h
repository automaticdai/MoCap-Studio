#pragma once

#include <string>
#include <vector>

namespace mocap {

struct CaptureConfig {
    int target_fps = 60;
    std::string sync_mode = "software";
    double max_sync_skew_ms = 5.0;
};

struct CameraConfig {
    std::string id;
    std::string type = "usb";    // "usb", "ip", "file"
    int device_index = 0;
    std::string url;             // for IP cameras
    std::string file_path;       // for file cameras
    int resolution_width = 1920;
    int resolution_height = 1080;
    std::string intrinsics_file;
    std::string extrinsics_file;
};

struct PoseEstimationConfig {
    std::string backend = "onnxruntime";
    std::string model = "rtmpose_l_body25.onnx";
    std::string device = "cuda:0";
    float detection_threshold = 0.5f;
    float keypoint_threshold = 0.3f;
};

struct TriangulationConfig {
    int min_views = 2;
    bool ransac_enabled = true;
    float ransac_threshold_px = 5.0f;
    std::string temporal_filter = "butterworth";
    float filter_cutoff_hz = 6.0f;
};

struct SkeletonConfig {
    std::string definition = "body_25";
    std::string ik_solver = "analytical";
    bool joint_limits_enabled = true;
};

struct GuiConfig {
    int canvas_fps = 60;
    std::vector<std::string> default_render_layers = {"grid", "markers", "skeleton"};
    std::string colour_palette = "oklab_12";
};

struct AppConfig {
    CaptureConfig capture;
    std::vector<CameraConfig> cameras;
    PoseEstimationConfig pose_estimation;
    TriangulationConfig triangulation;
    SkeletonConfig skeleton;
    GuiConfig gui;

    static AppConfig load(const std::string& path);
    void save(const std::string& path) const;
};

}  // namespace mocap
