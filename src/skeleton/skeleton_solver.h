#pragma once

#include <vector>
#include "core/types.h"
#include "core/skeleton_definition.h"

namespace mocap {

class SkeletonSolver {
public:
    enum class SolverType { Analytical, Optimisation };

    void setSkeletonDefinition(const SkeletonDefinition& def);
    void setSolverType(SolverType type);
    void setJointLimitsEnabled(bool enabled);

    // Solve for a single frame
    SkeletonPose solve(const Pose3D& pose3d);

    // Batch solve
    std::vector<SkeletonPose> solveBatch(const std::vector<Pose3D>& poses);

private:
    SkeletonDefinition skeleton_def_;
    SolverType solver_type_ = SolverType::Analytical;
    bool joint_limits_enabled_ = true;

    // Map marker index to 3D position
    Vec3f getMarkerPosition(const Pose3D& pose, int keypoint_index) const;
    bool hasMarker(const Pose3D& pose, int keypoint_index) const;

    // Compute root (pelvis) position and orientation
    std::pair<Vec3f, Quaternion> computeRootTransform(const Pose3D& pose3d);

    // Analytical IK for a 2-bone chain (e.g., shoulder-elbow-wrist)
    std::pair<Quaternion, Quaternion> solveIK2Bone(
        const Vec3f& root_pos, const Vec3f& mid_pos, const Vec3f& end_pos,
        const JointDef& root_joint, const JointDef& mid_joint
    );

    // Single joint look-at rotation
    Quaternion solveLookAt(
        const Vec3f& from_pos, const Vec3f& to_pos,
        const Vec3f& rest_direction
    );

    // Optimisation-based IK
    void solveIKOptimisation(SkeletonPose& pose, const Pose3D& markers);

    // Clamp rotation to joint limits
    Quaternion clampToLimits(const Quaternion& rotation, const JointLimits& limits);

public:
    // Convert quaternion to Euler angles (XYZ order, degrees)
    static Vec3f quaternionToEuler(const Quaternion& q);
};

}  // namespace mocap
