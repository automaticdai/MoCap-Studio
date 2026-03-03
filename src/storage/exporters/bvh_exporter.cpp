#include "storage/exporters/bvh_exporter.h"
#include <fstream>
#include <stdexcept>
#include <iomanip>
#include <spdlog/spdlog.h>

namespace mocap {

std::string BvhExporter::indent(int depth) {
    return std::string(depth * 2, ' ');
}

void BvhExporter::writeJointHierarchy(
    std::ofstream& f,
    const SkeletonDefinition& skeleton,
    int joint_index,
    int depth)
{
    const auto& joint = skeleton.joint(joint_index);
    auto children = skeleton.childrenOf(joint_index);

    // Offset in centimetres (BVH convention)
    float ox = joint.rest_offset.x() * 100.0f;
    float oy = joint.rest_offset.y() * 100.0f;
    float oz = joint.rest_offset.z() * 100.0f;

    if (joint.parent == -1) {
        f << "ROOT " << joint.name << "\n";
    } else {
        f << indent(depth) << "JOINT " << joint.name << "\n";
    }

    f << indent(depth) << "{\n";
    f << indent(depth + 1) << "OFFSET " << std::fixed << std::setprecision(4)
      << ox << " " << oy << " " << oz << "\n";

    if (joint.parent == -1) {
        // Root has 6 channels: position XYZ + rotation ZYX
        f << indent(depth + 1) << "CHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n";
    } else {
        f << indent(depth + 1) << "CHANNELS 3 Zrotation Xrotation Yrotation\n";
    }

    if (children.empty()) {
        // End site
        f << indent(depth + 1) << "End Site\n";
        f << indent(depth + 1) << "{\n";
        f << indent(depth + 2) << "OFFSET 0.0000 0.0000 0.0000\n";
        f << indent(depth + 1) << "}\n";
    } else {
        for (int child : children) {
            writeJointHierarchy(f, skeleton, child, depth + 1);
        }
    }

    f << indent(depth) << "}\n";
}

void BvhExporter::exportSkeleton(
    const std::string& path,
    const SkeletonDefinition& skeleton,
    const std::vector<std::pair<double, std::vector<SkeletonPose>>>& frames,
    double fps,
    int person_id)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Cannot open BVH file: " + path);

    f << std::fixed << std::setprecision(4);

    // --- HIERARCHY ---
    f << "HIERARCHY\n";

    // Find root joint (parent == -1)
    int root_idx = -1;
    for (int i = 0; i < skeleton.jointCount(); ++i) {
        if (skeleton.joint(i).parent == -1) {
            root_idx = i;
            break;
        }
    }

    if (root_idx < 0) {
        throw std::runtime_error("No root joint found in skeleton definition");
    }

    writeJointHierarchy(f, skeleton, root_idx, 0);

    // --- MOTION ---
    // Count frames for the specified person
    int frame_count = 0;
    for (const auto& [ts, poses] : frames) {
        for (const auto& pose : poses) {
            if (pose.global_person_id == person_id) {
                frame_count++;
                break;
            }
        }
    }

    f << "MOTION\n";
    f << "Frames: " << frame_count << "\n";
    f << "Frame Time: " << std::setprecision(6) << (1.0 / fps) << "\n";

    // Collect joint traversal order (same as hierarchy writing order)
    std::vector<int> joint_order;
    std::function<void(int)> collectOrder = [&](int idx) {
        joint_order.push_back(idx);
        for (int child : skeleton.childrenOf(idx)) {
            collectOrder(child);
        }
    };
    collectOrder(root_idx);

    // Write frame data
    for (const auto& [ts, poses] : frames) {
        const SkeletonPose* target = nullptr;
        for (const auto& pose : poses) {
            if (pose.global_person_id == person_id) {
                target = &pose;
                break;
            }
        }
        if (!target) continue;

        for (int ji : joint_order) {
            const auto& jd = skeleton.joint(ji);

            if (jd.parent == -1) {
                // Root: position (cm) + rotation
                f << target->root_position.x() * 100.0f << " "
                  << target->root_position.y() * 100.0f << " "
                  << target->root_position.z() * 100.0f << " ";
            }

            // Find rotation for this joint
            Vec3f euler = Vec3f::Zero();
            if (ji < static_cast<int>(target->joint_rotations.size())) {
                euler = target->joint_rotations[ji].euler_xyz;
            }

            // BVH uses ZXY order for rotation channels
            f << euler.z() << " " << euler.x() << " " << euler.y() << " ";
        }
        f << "\n";
    }

    spdlog::info("Exported {} frames to BVH: {}", frame_count, path);
}

}  // namespace mocap
