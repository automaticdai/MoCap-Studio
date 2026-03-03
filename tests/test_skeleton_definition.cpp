#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include "core/skeleton_definition.h"

using namespace mocap;

TEST(SkeletonDefinitionTest, DefaultBody25JointCount) {
    auto def = SkeletonDefinition::defaultBody25();
    EXPECT_EQ(def.jointCount(), 25);
}

TEST(SkeletonDefinitionTest, DefaultBody25Name) {
    auto def = SkeletonDefinition::defaultBody25();
    EXPECT_EQ(def.name, "body_25");
}

TEST(SkeletonDefinitionTest, RootJoint) {
    auto def = SkeletonDefinition::defaultBody25();
    // Neck (index 1) is the root
    EXPECT_EQ(def.joint(1).parent, -1);
    EXPECT_EQ(def.joint(1).name, "neck");
}

TEST(SkeletonDefinitionTest, ParentRelationships) {
    auto def = SkeletonDefinition::defaultBody25();

    // Nose's parent is neck
    EXPECT_EQ(def.joint(0).parent, 1);

    // Right shoulder's parent is neck
    EXPECT_EQ(def.joint(2).parent, 1);

    // Right elbow's parent is right shoulder
    EXPECT_EQ(def.joint(3).parent, 2);

    // Right knee's parent is right hip
    EXPECT_EQ(def.joint(10).parent, 9);
}

TEST(SkeletonDefinitionTest, ChildrenOf) {
    auto def = SkeletonDefinition::defaultBody25();

    // Neck's children: nose(0), r_shoulder(2), l_shoulder(5), mid_hip(8)
    auto neck_children = def.childrenOf(1);
    EXPECT_EQ(neck_children.size(), 4u);

    // Mid hip's children: right_hip(9), left_hip(12)
    auto hip_children = def.childrenOf(8);
    EXPECT_EQ(hip_children.size(), 2u);
}

TEST(SkeletonDefinitionTest, JointNames) {
    auto def = SkeletonDefinition::defaultBody25();
    EXPECT_EQ(def.joint(0).name, "nose");
    EXPECT_EQ(def.joint(4).name, "right_wrist");
    EXPECT_EQ(def.joint(7).name, "left_wrist");
    EXPECT_EQ(def.joint(8).name, "mid_hip");
    EXPECT_EQ(def.joint(24).name, "right_heel");
}

TEST(SkeletonDefinitionTest, KeypointToJointMap) {
    auto def = SkeletonDefinition::defaultBody25();
    EXPECT_EQ(def.keypoint_to_joint_map.size(), 25u);
    // 1:1 mapping for BODY_25
    for (int i = 0; i < 25; ++i) {
        EXPECT_EQ(def.keypoint_to_joint_map.at(i), i);
    }
}

TEST(SkeletonDefinitionTest, RestOffsets) {
    auto def = SkeletonDefinition::defaultBody25();

    // Right shoulder offset from neck should be negative x (right side)
    EXPECT_LT(def.joint(2).rest_offset.x(), 0.0f);

    // Left shoulder offset from neck should be positive x
    EXPECT_GT(def.joint(5).rest_offset.x(), 0.0f);

    // Mid hip offset from neck should be negative y (downward)
    EXPECT_LT(def.joint(8).rest_offset.y(), 0.0f);
}

TEST(SkeletonDefinitionTest, LoadFromJson) {
    // Create a temporary JSON file
    auto temp_path = std::filesystem::temp_directory_path() / "test_skeleton.json";

    nlohmann::json j;
    j["name"] = "test_skel";
    j["joints"] = nlohmann::json::array();

    nlohmann::json root_joint;
    root_joint["index"] = 0;
    root_joint["name"] = "root";
    root_joint["parent"] = -1;
    root_joint["rest_offset"] = {0.0, 0.0, 0.0};
    root_joint["twist_axis"] = {0, 1, 0};
    root_joint["limits"] = {{"min", {-90, -90, -90}}, {"max", {90, 90, 90}}};
    j["joints"].push_back(root_joint);

    nlohmann::json child_joint;
    child_joint["index"] = 1;
    child_joint["name"] = "child";
    child_joint["parent"] = 0;
    child_joint["rest_offset"] = {0.0, 0.5, 0.0};
    child_joint["twist_axis"] = {1, 0, 0};
    child_joint["limits"] = {{"min", {-45, -45, -45}}, {"max", {45, 45, 45}}};
    j["joints"].push_back(child_joint);

    j["keypoint_to_joint_map"] = {{"0", 0}, {"1", 1}};

    {
        std::ofstream f(temp_path.string());
        f << j.dump(2);
    }

    auto def = SkeletonDefinition::loadFromJson(temp_path.string());
    EXPECT_EQ(def.name, "test_skel");
    EXPECT_EQ(def.jointCount(), 2);
    EXPECT_EQ(def.joint(0).name, "root");
    EXPECT_EQ(def.joint(0).parent, -1);
    EXPECT_EQ(def.joint(1).name, "child");
    EXPECT_EQ(def.joint(1).parent, 0);
    EXPECT_FLOAT_EQ(def.joint(1).rest_offset.y(), 0.5f);
    EXPECT_FLOAT_EQ(def.joint(0).limits.min_euler.x(), -90.0f);
    EXPECT_FLOAT_EQ(def.joint(0).limits.max_euler.x(), 90.0f);
    EXPECT_EQ(def.keypoint_to_joint_map.at(0), 0);
    EXPECT_EQ(def.keypoint_to_joint_map.at(1), 1);

    std::filesystem::remove(temp_path);
}
