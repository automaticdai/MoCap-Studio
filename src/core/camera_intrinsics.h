#pragma once

#include <string>
#include <opencv2/core.hpp>

namespace mocap {

struct CameraIntrinsics {
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    cv::Mat distortion_coeffs;  // 1x5 or 1x8
    cv::Size image_size;

    cv::Mat cameraMatrix() const;
    cv::Point2f undistort(const cv::Point2f& pt) const;

    static CameraIntrinsics loadFromYaml(const std::string& path);
    void saveToYaml(const std::string& path) const;
};

struct CameraExtrinsics {
    cv::Mat rotation;      // 3x3 rotation matrix (world to camera)
    cv::Vec3d translation; // translation vector (world to camera)

    CameraExtrinsics();

    cv::Mat projectionMatrix(const CameraIntrinsics& intr) const;

    static CameraExtrinsics loadFromJson(const std::string& path);
    void saveToJson(const std::string& path) const;
};

}  // namespace mocap
