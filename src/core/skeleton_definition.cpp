#include "core/skeleton_definition.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <stdexcept>

namespace mocap {

SkeletonDefinition SkeletonDefinition::loadFromJson(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        throw std::runtime_error("Failed to open skeleton definition: " + path);
    }

    nlohmann::json j;
    f >> j;

    SkeletonDefinition def;
    def.name = j.value("name", "unnamed");

    for (const auto& jj : j["joints"]) {
        JointDef jd;
        jd.index = jj["index"].get<int>();
        jd.name = jj["name"].get<std::string>();
        jd.parent = jj["parent"].get<int>();

        auto& ro = jj["rest_offset"];
        jd.rest_offset = Vec3f(ro[0].get<float>(), ro[1].get<float>(), ro[2].get<float>());

        auto& ta = jj["twist_axis"];
        jd.twist_axis = Vec3f(ta[0].get<float>(), ta[1].get<float>(), ta[2].get<float>());

        if (jj.contains("limits")) {
            auto& lim = jj["limits"];
            auto& mn = lim["min"];
            auto& mx = lim["max"];
            jd.limits.min_euler = Vec3f(mn[0].get<float>(), mn[1].get<float>(), mn[2].get<float>());
            jd.limits.max_euler = Vec3f(mx[0].get<float>(), mx[1].get<float>(), mx[2].get<float>());
        }

        def.joints.push_back(jd);
    }

    if (j.contains("keypoint_to_joint_map")) {
        for (auto& [key, val] : j["keypoint_to_joint_map"].items()) {
            def.keypoint_to_joint_map[std::stoi(key)] = val.get<int>();
        }
    }

    return def;
}

SkeletonDefinition SkeletonDefinition::defaultBody25() {
    SkeletonDefinition def;
    def.name = "body_25";

    struct JointInfo {
        int index; const char* name; int parent;
        float ox, oy, oz;  // rest offset
        float tx, ty, tz;  // twist axis
    };

    // BODY_25 topology matching OpenPose
    static const JointInfo joints[] = {
        { 0,  "nose",            1,   0.00f,  0.08f,  0.06f,  0,1,0},
        { 1,  "neck",           -1,   0.00f,  0.00f,  0.00f,  0,1,0},
        { 2,  "right_shoulder",  1,  -0.18f, -0.02f,  0.00f,  1,0,0},
        { 3,  "right_elbow",     2,  -0.28f,  0.00f,  0.00f,  0,0,1},
        { 4,  "right_wrist",     3,  -0.25f,  0.00f,  0.00f,  0,0,1},
        { 5,  "left_shoulder",   1,   0.18f, -0.02f,  0.00f,  1,0,0},
        { 6,  "left_elbow",      5,   0.28f,  0.00f,  0.00f,  0,0,1},
        { 7,  "left_wrist",      6,   0.25f,  0.00f,  0.00f,  0,0,1},
        { 8,  "mid_hip",         1,   0.00f, -0.50f,  0.00f,  0,1,0},
        { 9,  "right_hip",       8,  -0.10f,  0.00f,  0.00f,  1,0,0},
        {10,  "right_knee",      9,   0.00f, -0.42f,  0.00f,  1,0,0},
        {11,  "right_ankle",    10,   0.00f, -0.40f,  0.00f,  1,0,0},
        {12,  "left_hip",        8,   0.10f,  0.00f,  0.00f,  1,0,0},
        {13,  "left_knee",      12,   0.00f, -0.42f,  0.00f,  1,0,0},
        {14,  "left_ankle",     13,   0.00f, -0.40f,  0.00f,  1,0,0},
        {15,  "right_eye",       0,  -0.03f,  0.03f,  0.02f,  0,1,0},
        {16,  "left_eye",        0,   0.03f,  0.03f,  0.02f,  0,1,0},
        {17,  "right_ear",       0,  -0.06f,  0.01f, -0.02f,  0,1,0},
        {18,  "left_ear",        0,   0.06f,  0.01f, -0.02f,  0,1,0},
        {19,  "left_big_toe",   14,   0.02f, -0.05f,  0.10f,  1,0,0},
        {20,  "left_small_toe", 14,  -0.02f, -0.05f,  0.10f,  1,0,0},
        {21,  "left_heel",      14,   0.00f, -0.05f, -0.04f,  1,0,0},
        {22,  "right_big_toe",  11,   0.02f, -0.05f,  0.10f,  1,0,0},
        {23,  "right_small_toe",11,  -0.02f, -0.05f,  0.10f,  1,0,0},
        {24,  "right_heel",     11,   0.00f, -0.05f, -0.04f,  1,0,0},
    };

    for (const auto& ji : joints) {
        JointDef jd;
        jd.index = ji.index;
        jd.name = ji.name;
        jd.parent = ji.parent;
        jd.rest_offset = Vec3f(ji.ox, ji.oy, ji.oz);
        jd.twist_axis = Vec3f(ji.tx, ji.ty, ji.tz);
        def.joints.push_back(jd);
    }

    // 1:1 keypoint to joint mapping for BODY_25
    for (int i = 0; i < 25; ++i) {
        def.keypoint_to_joint_map[i] = i;
    }

    return def;
}

int SkeletonDefinition::jointCount() const {
    return static_cast<int>(joints.size());
}

const JointDef& SkeletonDefinition::joint(int index) const {
    return joints.at(index);
}

std::vector<int> SkeletonDefinition::childrenOf(int joint_index) const {
    std::vector<int> children;
    for (const auto& j : joints) {
        if (j.parent == joint_index) {
            children.push_back(j.index);
        }
    }
    return children;
}

}  // namespace mocap
