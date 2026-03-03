#pragma once

#include <vector>
#include <string>
#include <utility>
#include <opencv2/core.hpp>
#include "core/types.h"
#include "core/camera_intrinsics.h"
#include "pose/person_tracker.h"

namespace mocap {

class Triangulator {
public:
    struct CameraView {
        CameraIntrinsics intrinsics;
        CameraExtrinsics extrinsics;
        std::string camera_id;
    };

    void setCameras(const std::vector<CameraView>& cameras);

    // Triangulate all keypoints for a set of tracked persons
    std::vector<Pose3D> triangulate(
        const std::vector<PersonTracker::TrackedPerson2D>& detections,
        double timestamp
    );

    // Triangulate a single 3D point from 2D observations across cameras
    std::pair<Vec3f, float> triangulateSinglePoint(
        const std::vector<std::pair<int, cv::Point2f>>& observations
    ) const;

    void setMinViews(int min_views);
    void setRansacEnabled(bool enabled);
    void setRansacThreshold(float threshold_px);

private:
    std::vector<CameraView> cameras_;
    int min_views_ = 2;
    bool ransac_enabled_ = true;
    float ransac_threshold_px_ = 5.0f;

    // Get camera index by id
    int cameraIndex(const std::string& camera_id) const;

    // Compute projection matrix for a camera
    cv::Mat projectionMatrix(int camera_idx) const;

    // DLT triangulation
    Vec3f dlt(const std::vector<std::pair<cv::Mat, cv::Point2f>>& proj_and_points) const;

    // RANSAC wrapper around DLT
    Vec3f ransacTriangulate(
        const std::vector<std::pair<int, cv::Point2f>>& observations,
        float& reprojection_error
    ) const;

    // Compute reprojection error for a 3D point
    float reprojectionError(
        const Vec3f& point3d,
        int camera_idx,
        const cv::Point2f& observed
    ) const;
};

}  // namespace mocap
