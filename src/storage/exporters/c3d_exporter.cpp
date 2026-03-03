#include "storage/exporters/c3d_exporter.h"
#include <ezc3d/ezc3d_all.h>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <set>

namespace mocap {

void C3dExporter::exportPose3D(
    const std::string& path,
    const std::vector<std::pair<double, std::vector<Pose3D>>>& frames,
    double fps,
    int person_id)
{
    if (frames.empty()) {
        throw std::runtime_error("No frames to export");
    }

    // Collect all unique marker names across all frames
    std::vector<std::string> marker_names;
    std::set<std::string> seen;

    for (const auto& [ts, poses] : frames) {
        for (const auto& pose : poses) {
            if (person_id >= 0 && pose.global_person_id != person_id) continue;
            for (const auto& m : pose.markers) {
                std::string label = (person_id < 0)
                    ? "P" + std::to_string(pose.global_person_id) + "_" + m.name
                    : m.name;
                if (seen.insert(label).second) {
                    marker_names.push_back(label);
                }
            }
        }
    }

    if (marker_names.empty()) {
        throw std::runtime_error("No markers found for export");
    }

    // Build marker name to index map
    std::unordered_map<std::string, int> name_to_idx;
    for (int i = 0; i < static_cast<int>(marker_names.size()); ++i) {
        name_to_idx[marker_names[i]] = i;
    }

    // Create C3D object
    ezc3d::c3d c3d;

    // Set parameters
    c3d.parameter("POINT:RATE").set(static_cast<float>(fps));
    c3d.parameter("POINT:LABELS").set(marker_names);

    // Add frames
    for (const auto& [ts, poses] : frames) {
        ezc3d::DataNS::Frame frame;
        ezc3d::DataNS::Points3dNS::Points pts;

        // Initialize all points as empty
        for (size_t i = 0; i < marker_names.size(); ++i) {
            ezc3d::DataNS::Points3dNS::Point pt;
            pt.x(0.0);
            pt.y(0.0);
            pt.z(0.0);
            // Residual < 0 means point is not visible
            pt.residual(-1.0);
            pts.point(pt);
        }

        // Fill in observed markers
        for (const auto& pose : poses) {
            if (person_id >= 0 && pose.global_person_id != person_id) continue;

            for (const auto& m : pose.markers) {
                std::string label = (person_id < 0)
                    ? "P" + std::to_string(pose.global_person_id) + "_" + m.name
                    : m.name;

                auto it = name_to_idx.find(label);
                if (it == name_to_idx.end()) continue;

                ezc3d::DataNS::Points3dNS::Point pt;
                // C3D uses millimetres
                pt.x(static_cast<double>(m.position.x()) * 1000.0);
                pt.y(static_cast<double>(m.position.y()) * 1000.0);
                pt.z(static_cast<double>(m.position.z()) * 1000.0);
                pt.residual(static_cast<double>(1.0f - m.confidence));
                pts.point(pt, static_cast<size_t>(it->second));
            }
        }

        frame.add(pts);
        c3d.frame(frame);
    }

    c3d.write(path);
    spdlog::info("Exported {} frames with {} markers to C3D: {}",
                 frames.size(), marker_names.size(), path);
}

}  // namespace mocap
