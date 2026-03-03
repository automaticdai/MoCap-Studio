#include "skeleton/skeleton_solver.h"
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace mocap {

void SkeletonSolver::setSkeletonDefinition(const SkeletonDefinition& def) {
    skeleton_def_ = def;
}

void SkeletonSolver::setSolverType(SolverType type) {
    solver_type_ = type;
}

void SkeletonSolver::setJointLimitsEnabled(bool enabled) {
    joint_limits_enabled_ = enabled;
}

Vec3f SkeletonSolver::getMarkerPosition(const Pose3D& pose, int keypoint_index) const {
    for (const auto& m : pose.markers) {
        if (m.index == keypoint_index) return m.position;
    }
    return Vec3f::Zero();
}

bool SkeletonSolver::hasMarker(const Pose3D& pose, int keypoint_index) const {
    for (const auto& m : pose.markers) {
        if (m.index == keypoint_index && m.confidence > 0.1f) return true;
    }
    return false;
}

std::pair<Vec3f, Quaternion> SkeletonSolver::computeRootTransform(const Pose3D& pose3d) {
    // Root is the neck (joint 1 in BODY_25)
    // Use mid_hip (8) and neck (1) to establish orientation
    // Use left_hip (12) and right_hip (9) for lateral direction

    Vec3f root_pos = Vec3f::Zero();
    Quaternion root_rot = Quaternion::Identity();

    // Try to find neck position
    if (hasMarker(pose3d, 1)) {
        root_pos = getMarkerPosition(pose3d, 1);
    } else if (hasMarker(pose3d, 8)) {
        // Fall back to mid-hip
        root_pos = getMarkerPosition(pose3d, 8);
    }

    // Compute orientation from hip and shoulder markers
    bool has_rhip = hasMarker(pose3d, 9);
    bool has_lhip = hasMarker(pose3d, 12);
    bool has_neck = hasMarker(pose3d, 1);
    bool has_midhip = hasMarker(pose3d, 8);

    if (has_rhip && has_lhip && (has_neck || has_midhip)) {
        Vec3f r_hip = getMarkerPosition(pose3d, 9);
        Vec3f l_hip = getMarkerPosition(pose3d, 12);
        Vec3f up_ref = has_neck ? getMarkerPosition(pose3d, 1) : root_pos;
        Vec3f hip_mid = (r_hip + l_hip) * 0.5f;

        Vec3f lateral = (l_hip - r_hip).normalized();  // X axis (left)
        Vec3f up = (up_ref - hip_mid).normalized();     // Y axis (up)
        Vec3f forward = lateral.cross(up).normalized(); // Z axis (forward)
        up = forward.cross(lateral).normalized();       // re-orthogonalize

        Eigen::Matrix3f rot_mat;
        rot_mat.col(0) = lateral;
        rot_mat.col(1) = up;
        rot_mat.col(2) = forward;

        root_rot = Quaternion(rot_mat);
    }

    return {root_pos, root_rot};
}

std::pair<Quaternion, Quaternion> SkeletonSolver::solveIK2Bone(
    const Vec3f& root_pos, const Vec3f& mid_pos, const Vec3f& end_pos,
    const JointDef& root_joint, const JointDef& mid_joint
) {
    Vec3f upper = mid_pos - root_pos;
    Vec3f lower = end_pos - mid_pos;

    float upper_len = upper.norm();
    float lower_len = lower.norm();

    if (upper_len < 1e-6f || lower_len < 1e-6f) {
        return {Quaternion::Identity(), Quaternion::Identity()};
    }

    // Root joint rotation: rotate rest direction to point at mid
    Vec3f rest_dir = root_joint.rest_offset.normalized();
    if (rest_dir.norm() < 0.5f) rest_dir = Vec3f(1, 0, 0);

    Vec3f target_dir = upper.normalized();
    Quaternion root_rot = Quaternion::FromTwoVectors(rest_dir, target_dir);

    // Mid joint rotation: angle between upper and lower bones
    Vec3f mid_rest_dir = mid_joint.rest_offset.normalized();
    if (mid_rest_dir.norm() < 0.5f) mid_rest_dir = rest_dir;

    // The mid joint rotation in the local frame of the upper bone
    Vec3f local_lower = root_rot.inverse() * lower.normalized();
    Quaternion mid_rot = Quaternion::FromTwoVectors(mid_rest_dir, local_lower);

    // Clamp if needed
    if (joint_limits_enabled_) {
        root_rot = clampToLimits(root_rot, root_joint.limits);
        mid_rot = clampToLimits(mid_rot, mid_joint.limits);
    }

    return {root_rot, mid_rot};
}

Quaternion SkeletonSolver::solveLookAt(
    const Vec3f& from_pos, const Vec3f& to_pos,
    const Vec3f& rest_direction
) {
    Vec3f dir = (to_pos - from_pos).normalized();
    Vec3f rest = rest_direction.normalized();

    if (rest.norm() < 0.5f) rest = Vec3f(0, 1, 0);
    if (dir.norm() < 0.5f) return Quaternion::Identity();

    return Quaternion::FromTwoVectors(rest, dir);
}

SkeletonPose SkeletonSolver::solve(const Pose3D& pose3d) {
    SkeletonPose result;
    result.global_person_id = pose3d.global_person_id;
    result.timestamp = pose3d.timestamp;

    auto [root_pos, root_rot] = computeRootTransform(pose3d);
    result.root_position = root_pos;
    result.root_rotation = root_rot;

    // Initialize joint rotations
    result.joint_rotations.resize(skeleton_def_.jointCount());

    for (int i = 0; i < skeleton_def_.jointCount(); ++i) {
        const auto& jd = skeleton_def_.joint(i);
        result.joint_rotations[i].joint_name = jd.name;
        result.joint_rotations[i].joint_index = jd.index;
        result.joint_rotations[i].rotation = Quaternion::Identity();
        result.joint_rotations[i].euler_xyz = Vec3f::Zero();
    }

    if (solver_type_ == SolverType::Optimisation) {
        solveIKOptimisation(result, pose3d);
    } else {
        // Analytical solving for key chains

        // Right arm: shoulder(2) -> elbow(3) -> wrist(4)
        if (hasMarker(pose3d, 2) && hasMarker(pose3d, 3) && hasMarker(pose3d, 4)) {
            auto [shoulder_rot, elbow_rot] = solveIK2Bone(
                getMarkerPosition(pose3d, 2),
                getMarkerPosition(pose3d, 3),
                getMarkerPosition(pose3d, 4),
                skeleton_def_.joint(2), skeleton_def_.joint(3)
            );
            result.joint_rotations[2].rotation = shoulder_rot;
            result.joint_rotations[3].rotation = elbow_rot;
        }

        // Left arm: shoulder(5) -> elbow(6) -> wrist(7)
        if (hasMarker(pose3d, 5) && hasMarker(pose3d, 6) && hasMarker(pose3d, 7)) {
            auto [shoulder_rot, elbow_rot] = solveIK2Bone(
                getMarkerPosition(pose3d, 5),
                getMarkerPosition(pose3d, 6),
                getMarkerPosition(pose3d, 7),
                skeleton_def_.joint(5), skeleton_def_.joint(6)
            );
            result.joint_rotations[5].rotation = shoulder_rot;
            result.joint_rotations[6].rotation = elbow_rot;
        }

        // Right leg: hip(9) -> knee(10) -> ankle(11)
        if (hasMarker(pose3d, 9) && hasMarker(pose3d, 10) && hasMarker(pose3d, 11)) {
            auto [hip_rot, knee_rot] = solveIK2Bone(
                getMarkerPosition(pose3d, 9),
                getMarkerPosition(pose3d, 10),
                getMarkerPosition(pose3d, 11),
                skeleton_def_.joint(9), skeleton_def_.joint(10)
            );
            result.joint_rotations[9].rotation = hip_rot;
            result.joint_rotations[10].rotation = knee_rot;
        }

        // Left leg: hip(12) -> knee(13) -> ankle(14)
        if (hasMarker(pose3d, 12) && hasMarker(pose3d, 13) && hasMarker(pose3d, 14)) {
            auto [hip_rot, knee_rot] = solveIK2Bone(
                getMarkerPosition(pose3d, 12),
                getMarkerPosition(pose3d, 13),
                getMarkerPosition(pose3d, 14),
                skeleton_def_.joint(12), skeleton_def_.joint(13)
            );
            result.joint_rotations[12].rotation = hip_rot;
            result.joint_rotations[13].rotation = knee_rot;
        }

        // Neck -> Nose look-at for head orientation
        if (hasMarker(pose3d, 1) && hasMarker(pose3d, 0)) {
            Quaternion head_rot = solveLookAt(
                getMarkerPosition(pose3d, 1),
                getMarkerPosition(pose3d, 0),
                skeleton_def_.joint(0).rest_offset
            );
            if (joint_limits_enabled_) {
                head_rot = clampToLimits(head_rot, skeleton_def_.joint(0).limits);
            }
            result.joint_rotations[0].rotation = head_rot;
        }

        // Mid-hip orientation
        if (hasMarker(pose3d, 1) && hasMarker(pose3d, 8)) {
            Quaternion spine_rot = solveLookAt(
                getMarkerPosition(pose3d, 8),
                getMarkerPosition(pose3d, 1),
                skeleton_def_.joint(8).rest_offset
            );
            if (joint_limits_enabled_) {
                spine_rot = clampToLimits(spine_rot, skeleton_def_.joint(8).limits);
            }
            result.joint_rotations[8].rotation = spine_rot;
        }
    }

    // Convert all rotations to Euler angles
    for (auto& jr : result.joint_rotations) {
        jr.euler_xyz = quaternionToEuler(jr.rotation);
    }

    return result;
}

std::vector<SkeletonPose> SkeletonSolver::solveBatch(const std::vector<Pose3D>& poses) {
    std::vector<SkeletonPose> results;
    results.reserve(poses.size());
    for (const auto& pose : poses) {
        results.push_back(solve(pose));
    }
    return results;
}

void SkeletonSolver::solveIKOptimisation(SkeletonPose& pose, const Pose3D& markers) {
    // Simple iterative optimisation: for each joint with a known marker,
    // compute the rotation that minimises the distance between
    // predicted and observed marker positions.

    constexpr int MAX_ITER = 10;
    constexpr float LEARNING_RATE = 0.5f;

    for (int iter = 0; iter < MAX_ITER; ++iter) {
        for (int j = 0; j < skeleton_def_.jointCount(); ++j) {
            const auto& jd = skeleton_def_.joint(j);

            // Find children with observed markers
            auto children = skeleton_def_.childrenOf(j);
            if (children.empty()) continue;

            for (int child_idx : children) {
                if (!hasMarker(markers, child_idx)) continue;

                Vec3f observed = getMarkerPosition(markers, child_idx);
                Vec3f parent_pos = hasMarker(markers, j)
                    ? getMarkerPosition(markers, j)
                    : pose.root_position;

                // Current predicted direction
                Vec3f rest_dir = jd.rest_offset.normalized();
                if (rest_dir.norm() < 0.5f) continue;

                Vec3f target_dir = (observed - parent_pos).normalized();
                if (target_dir.norm() < 0.5f) continue;

                Quaternion desired = Quaternion::FromTwoVectors(rest_dir, target_dir);

                // Blend with current rotation
                pose.joint_rotations[j].rotation =
                    pose.joint_rotations[j].rotation.slerp(LEARNING_RATE, desired);

                if (joint_limits_enabled_) {
                    pose.joint_rotations[j].rotation =
                        clampToLimits(pose.joint_rotations[j].rotation, jd.limits);
                }
            }
        }
    }
}

Quaternion SkeletonSolver::clampToLimits(const Quaternion& rotation, const JointLimits& limits) {
    Vec3f euler = quaternionToEuler(rotation);

    float deg2rad = static_cast<float>(M_PI / 180.0);

    euler.x() = std::clamp(euler.x(), limits.min_euler.x(), limits.max_euler.x());
    euler.y() = std::clamp(euler.y(), limits.min_euler.y(), limits.max_euler.y());
    euler.z() = std::clamp(euler.z(), limits.min_euler.z(), limits.max_euler.z());

    // Reconstruct quaternion from clamped Euler
    Quaternion qx(Eigen::AngleAxisf(euler.x() * deg2rad, Vec3f::UnitX()));
    Quaternion qy(Eigen::AngleAxisf(euler.y() * deg2rad, Vec3f::UnitY()));
    Quaternion qz(Eigen::AngleAxisf(euler.z() * deg2rad, Vec3f::UnitZ()));

    return qx * qy * qz;
}

Vec3f SkeletonSolver::quaternionToEuler(const Quaternion& q) {
    // Extract Euler angles (XYZ intrinsic = ZYX extrinsic) in degrees
    auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    float rad2deg = static_cast<float>(180.0 / M_PI);

    Vec3f degrees(euler.x() * rad2deg, euler.y() * rad2deg, euler.z() * rad2deg);

    // Normalize to [-180, 180]
    for (int i = 0; i < 3; ++i) {
        while (degrees[i] > 180.0f) degrees[i] -= 360.0f;
        while (degrees[i] < -180.0f) degrees[i] += 360.0f;
    }

    return degrees;
}

}  // namespace mocap
