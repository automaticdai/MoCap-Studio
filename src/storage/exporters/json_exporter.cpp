#include "storage/exporters/json_exporter.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace mocap {

void JsonExporter::exportRaw2D(const std::string& path,
    const std::vector<std::pair<double, std::vector<Raw2DPose>>>& frames)
{
    nlohmann::json root;
    root["layer"] = "raw_2d";
    root["frame_count"] = frames.size();

    auto& jframes = root["frames"];
    jframes = nlohmann::json::array();

    for (const auto& [timestamp, poses] : frames) {
        nlohmann::json jf;
        jf["timestamp"] = timestamp;

        auto& jposes = jf["poses"];
        jposes = nlohmann::json::array();

        for (const auto& pose : poses) {
            nlohmann::json jp;
            jp["person_id"] = pose.person_id;
            jp["confidence"] = pose.confidence;
            jp["bbox"] = {pose.bbox.x, pose.bbox.y, pose.bbox.width, pose.bbox.height};

            auto& jkps = jp["keypoints"];
            jkps = nlohmann::json::array();
            for (const auto& kp : pose.keypoints) {
                jkps.push_back({
                    {"name", kp.name}, {"index", kp.index},
                    {"x", kp.x}, {"y", kp.y}, {"confidence", kp.conf}
                });
            }
            jposes.push_back(jp);
        }
        jframes.push_back(jf);
    }

    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open JSON file: " + path);
    f << root.dump(2);
}

void JsonExporter::exportPose3D(const std::string& path,
    const std::vector<std::pair<double, std::vector<Pose3D>>>& frames)
{
    nlohmann::json root;
    root["layer"] = "pose_3d";
    root["frame_count"] = frames.size();

    auto& jframes = root["frames"];
    jframes = nlohmann::json::array();

    for (const auto& [timestamp, poses] : frames) {
        nlohmann::json jf;
        jf["timestamp"] = timestamp;

        auto& jposes = jf["poses"];
        jposes = nlohmann::json::array();

        for (const auto& pose : poses) {
            nlohmann::json jp;
            jp["global_person_id"] = pose.global_person_id;

            auto& jmarkers = jp["markers"];
            jmarkers = nlohmann::json::array();
            for (const auto& m : pose.markers) {
                jmarkers.push_back({
                    {"name", m.name}, {"index", m.index},
                    {"position", {m.position.x(), m.position.y(), m.position.z()}},
                    {"confidence", m.confidence}, {"n_views", m.n_views}
                });
            }
            jposes.push_back(jp);
        }
        jframes.push_back(jf);
    }

    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open JSON file: " + path);
    f << root.dump(2);
}

void JsonExporter::exportSkeleton(const std::string& path,
    const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames)
{
    nlohmann::json root;
    root["layer"] = "skeleton";
    root["frame_count"] = frames.size();

    auto& jframes = root["frames"];
    jframes = nlohmann::json::array();

    for (const auto& [timestamp, poses] : frames) {
        nlohmann::json jf;
        jf["timestamp"] = timestamp;

        auto& jposes = jf["poses"];
        jposes = nlohmann::json::array();

        for (const auto& pose : poses) {
            nlohmann::json jp;
            jp["global_person_id"] = pose.global_person_id;
            jp["root_position"] = {
                pose.root_position.x(), pose.root_position.y(), pose.root_position.z()
            };
            jp["root_rotation"] = {
                pose.root_rotation.w(), pose.root_rotation.x(),
                pose.root_rotation.y(), pose.root_rotation.z()
            };

            auto& jjoints = jp["joint_rotations"];
            jjoints = nlohmann::json::array();
            for (const auto& jr : pose.joint_rotations) {
                jjoints.push_back({
                    {"joint_name", jr.joint_name}, {"joint_index", jr.joint_index},
                    {"rotation", {jr.rotation.w(), jr.rotation.x(),
                                  jr.rotation.y(), jr.rotation.z()}},
                    {"euler_xyz", {jr.euler_xyz.x(), jr.euler_xyz.y(), jr.euler_xyz.z()}}
                });
            }
            jposes.push_back(jp);
        }
        jframes.push_back(jf);
    }

    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open JSON file: " + path);
    f << root.dump(2);
}

}  // namespace mocap
