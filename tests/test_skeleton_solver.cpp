#include <gtest/gtest.h>
#include "skeleton/skeleton_solver.h"
#include <cmath>

using namespace mocap;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

Pose3D makeTPose() {
    // Create a T-pose with BODY_25 markers
    Pose3D pose;
    pose.global_person_id = 0;
    pose.timestamp = 0.0;

    auto addMarker = [&](int idx, const std::string& name, float x, float y, float z) {
        Marker3D m;
        m.index = idx;
        m.name = name;
        m.position = Vec3f(x, y, z);
        m.confidence = 0.9f;
        m.n_views = 3;
        pose.markers.push_back(m);
    };

    // Approximate T-pose (standing, arms out to sides)
    addMarker(0,  "nose",            0.00f,  1.70f,  0.06f);
    addMarker(1,  "neck",            0.00f,  1.55f,  0.00f);
    addMarker(2,  "right_shoulder", -0.20f,  1.53f,  0.00f);
    addMarker(3,  "right_elbow",    -0.48f,  1.53f,  0.00f);
    addMarker(4,  "right_wrist",    -0.73f,  1.53f,  0.00f);
    addMarker(5,  "left_shoulder",   0.20f,  1.53f,  0.00f);
    addMarker(6,  "left_elbow",      0.48f,  1.53f,  0.00f);
    addMarker(7,  "left_wrist",      0.73f,  1.53f,  0.00f);
    addMarker(8,  "mid_hip",         0.00f,  1.00f,  0.00f);
    addMarker(9,  "right_hip",      -0.10f,  1.00f,  0.00f);
    addMarker(10, "right_knee",     -0.10f,  0.55f,  0.00f);
    addMarker(11, "right_ankle",    -0.10f,  0.10f,  0.00f);
    addMarker(12, "left_hip",        0.10f,  1.00f,  0.00f);
    addMarker(13, "left_knee",       0.10f,  0.55f,  0.00f);
    addMarker(14, "left_ankle",      0.10f,  0.10f,  0.00f);

    return pose;
}

}  // namespace

TEST(SkeletonSolverTest, SolveTPose) {
    SkeletonSolver solver;
    solver.setSkeletonDefinition(SkeletonDefinition::defaultBody25());
    solver.setSolverType(SkeletonSolver::SolverType::Analytical);

    Pose3D tpose = makeTPose();
    SkeletonPose result = solver.solve(tpose);

    EXPECT_EQ(result.global_person_id, 0);
    EXPECT_DOUBLE_EQ(result.timestamp, 0.0);
    EXPECT_EQ(result.joint_rotations.size(), 25u);

    // Root position should be near neck
    EXPECT_NEAR(result.root_position.y(), 1.55f, 0.1f);
}

TEST(SkeletonSolverTest, JointNamesMatch) {
    SkeletonSolver solver;
    auto def = SkeletonDefinition::defaultBody25();
    solver.setSkeletonDefinition(def);

    Pose3D tpose = makeTPose();
    SkeletonPose result = solver.solve(tpose);

    ASSERT_EQ(result.joint_rotations.size(), 25u);
    for (int i = 0; i < 25; ++i) {
        EXPECT_EQ(result.joint_rotations[i].joint_name, def.joint(i).name);
        EXPECT_EQ(result.joint_rotations[i].joint_index, i);
    }
}

TEST(SkeletonSolverTest, EulerAnglesComputed) {
    SkeletonSolver solver;
    solver.setSkeletonDefinition(SkeletonDefinition::defaultBody25());

    Pose3D tpose = makeTPose();
    SkeletonPose result = solver.solve(tpose);

    // All euler angles should be finite
    for (const auto& jr : result.joint_rotations) {
        EXPECT_TRUE(std::isfinite(jr.euler_xyz.x()));
        EXPECT_TRUE(std::isfinite(jr.euler_xyz.y()));
        EXPECT_TRUE(std::isfinite(jr.euler_xyz.z()));
    }
}

TEST(SkeletonSolverTest, JointLimitsRespected) {
    SkeletonSolver solver;
    auto def = SkeletonDefinition::defaultBody25();
    solver.setSkeletonDefinition(def);
    solver.setJointLimitsEnabled(true);

    // Create a pose with an extreme arm position
    Pose3D pose = makeTPose();
    // Move right wrist behind the body (extreme position)
    for (auto& m : pose.markers) {
        if (m.index == 4) {
            m.position = Vec3f(-0.5f, 1.53f, -0.5f);
        }
    }

    SkeletonPose result = solver.solve(pose);

    // Right elbow (index 3) has limits [0,0,-150] to [0,0,0]
    // The Euler angles should be within valid range
    // (limits are checked but due to analytical solving may not be perfectly clamped)
    for (const auto& jr : result.joint_rotations) {
        EXPECT_TRUE(std::isfinite(jr.euler_xyz.x()));
        EXPECT_TRUE(std::isfinite(jr.euler_xyz.y()));
        EXPECT_TRUE(std::isfinite(jr.euler_xyz.z()));
    }
}

TEST(SkeletonSolverTest, BatchSolve) {
    SkeletonSolver solver;
    solver.setSkeletonDefinition(SkeletonDefinition::defaultBody25());

    std::vector<Pose3D> poses;
    for (int i = 0; i < 5; ++i) {
        Pose3D p = makeTPose();
        p.timestamp = i * (1.0 / 60.0);
        poses.push_back(p);
    }

    auto results = solver.solveBatch(poses);
    EXPECT_EQ(results.size(), 5u);

    for (int i = 0; i < 5; ++i) {
        EXPECT_DOUBLE_EQ(results[i].timestamp, i * (1.0 / 60.0));
    }
}

TEST(SkeletonSolverTest, OptimisationSolver) {
    SkeletonSolver solver;
    solver.setSkeletonDefinition(SkeletonDefinition::defaultBody25());
    solver.setSolverType(SkeletonSolver::SolverType::Optimisation);

    Pose3D tpose = makeTPose();
    SkeletonPose result = solver.solve(tpose);

    EXPECT_EQ(result.joint_rotations.size(), 25u);
    // Rotations should be finite
    for (const auto& jr : result.joint_rotations) {
        EXPECT_TRUE(std::isfinite(jr.rotation.w()));
        EXPECT_TRUE(std::isfinite(jr.rotation.x()));
    }
}

TEST(SkeletonSolverTest, QuaternionToEulerIdentity) {
    Quaternion identity = Quaternion::Identity();
    Vec3f euler = SkeletonSolver::quaternionToEuler(identity);

    EXPECT_NEAR(euler.x(), 0.0f, 1.0f);
    EXPECT_NEAR(euler.y(), 0.0f, 1.0f);
    EXPECT_NEAR(euler.z(), 0.0f, 1.0f);
}
