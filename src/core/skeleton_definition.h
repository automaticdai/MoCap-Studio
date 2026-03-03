#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include "core/types.h"

namespace mocap {

struct JointLimits {
    Vec3f min_euler = Vec3f(-180.0f, -180.0f, -180.0f);  // degrees
    Vec3f max_euler = Vec3f(180.0f, 180.0f, 180.0f);     // degrees
};

struct JointDef {
    int index = 0;
    std::string name;
    int parent = -1;          // -1 for root
    Vec3f rest_offset = Vec3f::Zero();  // offset from parent in T-pose (metres)
    Vec3f twist_axis = Vec3f(0, 1, 0);  // primary rotation axis hint
    JointLimits limits;
};

class SkeletonDefinition {
public:
    std::string name;
    std::vector<JointDef> joints;
    std::unordered_map<int, int> keypoint_to_joint_map;

    static SkeletonDefinition loadFromJson(const std::string& path);
    static SkeletonDefinition defaultBody25();

    int jointCount() const;
    const JointDef& joint(int index) const;
    std::vector<int> childrenOf(int joint_index) const;
};

}  // namespace mocap
