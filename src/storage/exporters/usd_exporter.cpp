#include "storage/exporters/usd_exporter.h"
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <spdlog/spdlog.h>

namespace mocap {

void UsdExporter::exportSkeleton(
    const std::string& path,
    const SkeletonDefinition& skeleton,
    const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames,
    double fps,
    int person_id)
{
    // Write a basic USDA (text-based USD) file
    // Full USD SDK integration would use pxr::UsdStageRefPtr etc.
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open USD file: " + path);

    f << std::fixed << std::setprecision(6);
    f << "#usda 1.0\n";
    f << "(\n";
    f << "    defaultPrim = \"MoCapSkeleton\"\n";
    f << "    metersPerUnit = 1.0\n";
    f << "    upAxis = \"Y\"\n";
    f << "    startTimeCode = 0\n";

    double end_time = frames.empty() ? 0.0 : frames.size() - 1;
    f << "    endTimeCode = " << end_time << "\n";
    f << "    timeCodesPerSecond = " << fps << "\n";
    f << ")\n\n";

    f << "def Xform \"MoCapSkeleton\" (\n";
    f << "    kind = \"component\"\n";
    f << ")\n{\n";

    // Build joint paths
    std::vector<std::string> joint_paths(skeleton.jointCount());
    for (int i = 0; i < skeleton.jointCount(); ++i) {
        const auto& jd = skeleton.joint(i);
        if (jd.parent == -1) {
            joint_paths[i] = jd.name;
        } else {
            joint_paths[i] = joint_paths[jd.parent] + "/" + jd.name;
        }
    }

    // Write skeleton definition
    f << "    def Skeleton \"Skeleton\"\n";
    f << "    {\n";

    // Joints list
    f << "        uniform token[] joints = [";
    for (int i = 0; i < skeleton.jointCount(); ++i) {
        if (i > 0) f << ", ";
        f << "\"" << joint_paths[i] << "\"";
    }
    f << "]\n\n";

    // Rest transforms
    f << "        uniform matrix4d[] restTransforms = [";
    for (int i = 0; i < skeleton.jointCount(); ++i) {
        if (i > 0) f << ", ";
        const auto& jd = skeleton.joint(i);
        f << "( (1,0,0,0), (0,1,0,0), (0,0,1,0), ("
          << jd.rest_offset.x() << "," << jd.rest_offset.y() << ","
          << jd.rest_offset.z() << ",1) )";
    }
    f << "]\n\n";

    // Write animation as time samples on joint transforms
    if (!frames.empty()) {
        f << "        rel skel:animationSource = </MoCapSkeleton/Animation>\n";
    }

    f << "    }\n\n";

    // Write animation
    if (!frames.empty()) {
        f << "    def SkelAnimation \"Animation\"\n";
        f << "    {\n";

        // Joints reference
        f << "        uniform token[] joints = [";
        for (int i = 0; i < skeleton.jointCount(); ++i) {
            if (i > 0) f << ", ";
            f << "\"" << joint_paths[i] << "\"";
        }
        f << "]\n\n";

        // Write rotations as time samples
        f << "        quatf[] rotations.timeSamples = {\n";
        int frame_idx = 0;
        for (const auto& [ts, poses] : frames) {
            const SkeletonPose* target = nullptr;
            for (const auto& pose : poses) {
                if (pose.global_person_id == person_id) { target = &pose; break; }
            }
            if (!target) { frame_idx++; continue; }

            f << "            " << frame_idx << ": [";
            for (int j = 0; j < skeleton.jointCount(); ++j) {
                if (j > 0) f << ", ";
                Quaternion q = Quaternion::Identity();
                if (j < static_cast<int>(target->joint_rotations.size())) {
                    q = target->joint_rotations[j].rotation;
                }
                f << "(" << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ")";
            }
            f << "],\n";
            frame_idx++;
        }
        f << "        }\n\n";

        // Write translations as time samples (only root changes)
        f << "        float3[] translations.timeSamples = {\n";
        frame_idx = 0;
        for (const auto& [ts, poses] : frames) {
            const SkeletonPose* target = nullptr;
            for (const auto& pose : poses) {
                if (pose.global_person_id == person_id) { target = &pose; break; }
            }
            if (!target) { frame_idx++; continue; }

            f << "            " << frame_idx << ": [";
            for (int j = 0; j < skeleton.jointCount(); ++j) {
                if (j > 0) f << ", ";
                if (skeleton.joint(j).parent == -1) {
                    f << "(" << target->root_position.x() << ", "
                      << target->root_position.y() << ", "
                      << target->root_position.z() << ")";
                } else {
                    const auto& off = skeleton.joint(j).rest_offset;
                    f << "(" << off.x() << ", " << off.y() << ", " << off.z() << ")";
                }
            }
            f << "],\n";
            frame_idx++;
        }
        f << "        }\n";

        f << "    }\n";
    }

    f << "}\n";

    spdlog::info("Exported {} frames to USDA: {}", frames.size(), path);
}

}  // namespace mocap
