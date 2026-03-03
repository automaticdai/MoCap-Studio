#include "core/config.h"
#include <yaml-cpp/yaml.h>
#include <fstream>

namespace mocap {

AppConfig AppConfig::load(const std::string& path) {
    AppConfig config;
    YAML::Node root = YAML::LoadFile(path);

    // Capture
    if (auto node = root["capture"]) {
        if (node["target_fps"]) config.capture.target_fps = node["target_fps"].as<int>();
        if (node["sync_mode"]) config.capture.sync_mode = node["sync_mode"].as<std::string>();
        if (node["max_sync_skew_ms"]) config.capture.max_sync_skew_ms = node["max_sync_skew_ms"].as<double>();
    }

    // Cameras
    if (auto cameras_node = root["cameras"]) {
        for (const auto& cam : cameras_node) {
            CameraConfig cc;
            if (cam["id"]) cc.id = cam["id"].as<std::string>();
            if (cam["type"]) cc.type = cam["type"].as<std::string>();
            if (cam["device_index"]) cc.device_index = cam["device_index"].as<int>();
            if (cam["url"]) cc.url = cam["url"].as<std::string>();
            if (cam["file_path"]) cc.file_path = cam["file_path"].as<std::string>();
            if (cam["resolution"]) {
                auto res = cam["resolution"];
                if (res.IsSequence() && res.size() == 2) {
                    cc.resolution_width = res[0].as<int>();
                    cc.resolution_height = res[1].as<int>();
                }
            }
            if (cam["intrinsics_file"]) cc.intrinsics_file = cam["intrinsics_file"].as<std::string>();
            if (cam["extrinsics_file"]) cc.extrinsics_file = cam["extrinsics_file"].as<std::string>();
            config.cameras.push_back(cc);
        }
    }

    // Pose estimation
    if (auto node = root["pose_estimation"]) {
        if (node["backend"]) config.pose_estimation.backend = node["backend"].as<std::string>();
        if (node["model"]) config.pose_estimation.model = node["model"].as<std::string>();
        if (node["device"]) config.pose_estimation.device = node["device"].as<std::string>();
        if (node["detection_threshold"]) config.pose_estimation.detection_threshold = node["detection_threshold"].as<float>();
        if (node["keypoint_threshold"]) config.pose_estimation.keypoint_threshold = node["keypoint_threshold"].as<float>();
    }

    // Triangulation
    if (auto node = root["triangulation"]) {
        if (node["min_views"]) config.triangulation.min_views = node["min_views"].as<int>();
        if (node["ransac_enabled"]) config.triangulation.ransac_enabled = node["ransac_enabled"].as<bool>();
        if (node["ransac_threshold_px"]) config.triangulation.ransac_threshold_px = node["ransac_threshold_px"].as<float>();
        if (node["temporal_filter"]) config.triangulation.temporal_filter = node["temporal_filter"].as<std::string>();
        if (node["filter_cutoff_hz"]) config.triangulation.filter_cutoff_hz = node["filter_cutoff_hz"].as<float>();
    }

    // Skeleton
    if (auto node = root["skeleton"]) {
        if (node["definition"]) config.skeleton.definition = node["definition"].as<std::string>();
        if (node["ik_solver"]) config.skeleton.ik_solver = node["ik_solver"].as<std::string>();
        if (node["joint_limits_enabled"]) config.skeleton.joint_limits_enabled = node["joint_limits_enabled"].as<bool>();
    }

    // GUI
    if (auto node = root["gui"]) {
        if (node["canvas_fps"]) config.gui.canvas_fps = node["canvas_fps"].as<int>();
        if (node["default_render_layers"]) {
            config.gui.default_render_layers.clear();
            for (const auto& layer : node["default_render_layers"]) {
                config.gui.default_render_layers.push_back(layer.as<std::string>());
            }
        }
        if (node["colour_palette"]) config.gui.colour_palette = node["colour_palette"].as<std::string>();
    }

    return config;
}

void AppConfig::save(const std::string& path) const {
    YAML::Emitter out;
    out << YAML::BeginMap;

    // Capture
    out << YAML::Key << "capture" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "target_fps" << YAML::Value << capture.target_fps;
    out << YAML::Key << "sync_mode" << YAML::Value << capture.sync_mode;
    out << YAML::Key << "max_sync_skew_ms" << YAML::Value << capture.max_sync_skew_ms;
    out << YAML::EndMap;

    // Cameras
    out << YAML::Key << "cameras" << YAML::Value << YAML::BeginSeq;
    for (const auto& cam : cameras) {
        out << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << cam.id;
        out << YAML::Key << "type" << YAML::Value << cam.type;
        if (cam.type == "usb") {
            out << YAML::Key << "device_index" << YAML::Value << cam.device_index;
        } else if (cam.type == "ip") {
            out << YAML::Key << "url" << YAML::Value << cam.url;
        } else if (cam.type == "file") {
            out << YAML::Key << "file_path" << YAML::Value << cam.file_path;
        }
        out << YAML::Key << "resolution" << YAML::Value << YAML::Flow
            << YAML::BeginSeq << cam.resolution_width << cam.resolution_height << YAML::EndSeq;
        if (!cam.intrinsics_file.empty())
            out << YAML::Key << "intrinsics_file" << YAML::Value << cam.intrinsics_file;
        if (!cam.extrinsics_file.empty())
            out << YAML::Key << "extrinsics_file" << YAML::Value << cam.extrinsics_file;
        out << YAML::EndMap;
    }
    out << YAML::EndSeq;

    // Pose estimation
    out << YAML::Key << "pose_estimation" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "backend" << YAML::Value << pose_estimation.backend;
    out << YAML::Key << "model" << YAML::Value << pose_estimation.model;
    out << YAML::Key << "device" << YAML::Value << pose_estimation.device;
    out << YAML::Key << "detection_threshold" << YAML::Value << pose_estimation.detection_threshold;
    out << YAML::Key << "keypoint_threshold" << YAML::Value << pose_estimation.keypoint_threshold;
    out << YAML::EndMap;

    // Triangulation
    out << YAML::Key << "triangulation" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "min_views" << YAML::Value << triangulation.min_views;
    out << YAML::Key << "ransac_enabled" << YAML::Value << triangulation.ransac_enabled;
    out << YAML::Key << "ransac_threshold_px" << YAML::Value << triangulation.ransac_threshold_px;
    out << YAML::Key << "temporal_filter" << YAML::Value << triangulation.temporal_filter;
    out << YAML::Key << "filter_cutoff_hz" << YAML::Value << triangulation.filter_cutoff_hz;
    out << YAML::EndMap;

    // Skeleton
    out << YAML::Key << "skeleton" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "definition" << YAML::Value << skeleton.definition;
    out << YAML::Key << "ik_solver" << YAML::Value << skeleton.ik_solver;
    out << YAML::Key << "joint_limits_enabled" << YAML::Value << skeleton.joint_limits_enabled;
    out << YAML::EndMap;

    // GUI
    out << YAML::Key << "gui" << YAML::Value << YAML::BeginMap;
    out << YAML::Key << "canvas_fps" << YAML::Value << gui.canvas_fps;
    out << YAML::Key << "default_render_layers" << YAML::Value << YAML::Flow
        << YAML::BeginSeq;
    for (const auto& layer : gui.default_render_layers) {
        out << layer;
    }
    out << YAML::EndSeq;
    out << YAML::Key << "colour_palette" << YAML::Value << gui.colour_palette;
    out << YAML::EndMap;

    out << YAML::EndMap;

    std::ofstream fout(path);
    fout << out.c_str();
}

}  // namespace mocap
