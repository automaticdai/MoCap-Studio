#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace mocap {

using Vec3f = Eigen::Vector3f;
using Quaternion = Eigen::Quaternionf;

// --- L1: Raw 2D Pose Data ---

struct Rect {
    float x = 0.0f;
    float y = 0.0f;
    float width = 0.0f;
    float height = 0.0f;

    float area() const { return width * height; }
    float iou(const Rect& other) const;
};

struct Keypoint2D {
    std::string name;
    int index = 0;
    float x = 0.0f;
    float y = 0.0f;
    float conf = 0.0f;
};

struct Raw2DPose {
    int person_id = 0;
    std::vector<Keypoint2D> keypoints;
    Rect bbox;
    float confidence = 0.0f;
};

// --- L2: 3D Marker Data ---

struct Marker3D {
    std::string name;
    int index = 0;
    Vec3f position = Vec3f::Zero();
    float confidence = 0.0f;
    int n_views = 0;
};

struct Pose3D {
    int global_person_id = 0;
    double timestamp = 0.0;
    std::vector<Marker3D> markers;
};

// --- L3: Skeletal Data ---

struct JointRotation {
    std::string joint_name;
    int joint_index = 0;
    Quaternion rotation = Quaternion::Identity();
    Vec3f euler_xyz = Vec3f::Zero();  // degrees
};

struct SkeletonPose {
    int global_person_id = 0;
    double timestamp = 0.0;
    Vec3f root_position = Vec3f::Zero();
    Quaternion root_rotation = Quaternion::Identity();
    std::vector<JointRotation> joint_rotations;
};

// --- Capture Types ---

struct CapturedFrame {
    std::string camera_id;
    double timestamp = 0.0;  // seconds from session start
    int frame_number = 0;
    cv::Mat image;
};

struct FrameSet {
    double timestamp = 0.0;
    std::vector<CapturedFrame> frames;
};

// --- Binary Wire Format ---

struct FramePacket {
    static constexpr uint32_t MAGIC = 0x4D435030;  // "MCP0"

    uint32_t magic = MAGIC;
    uint8_t version = 1;
    uint8_t layer = 0;       // 1=Raw2D, 2=Pose3D, 3=Skeleton
    uint16_t person_id = 0;
    double timestamp = 0.0;
    uint16_t n_items = 0;
    std::vector<uint8_t> payload;

    bool isValid() const { return magic == MAGIC; }
};

// --- Inline Implementations ---

inline float Rect::iou(const Rect& other) const {
    float x1 = std::max(x, other.x);
    float y1 = std::max(y, other.y);
    float x2 = std::min(x + width, other.x + other.width);
    float y2 = std::min(y + height, other.y + other.height);

    float intersection = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
    float union_area = area() + other.area() - intersection;

    if (union_area <= 0.0f) return 0.0f;
    return intersection / union_area;
}

}  // namespace mocap
