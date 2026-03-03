#include "triangulation/triangulator.h"
#include <opencv2/calib3d.hpp>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <unordered_map>
#include <random>
#include <cmath>

namespace mocap {

void Triangulator::setCameras(const std::vector<CameraView>& cameras) {
    cameras_ = cameras;
}

void Triangulator::setMinViews(int min_views) {
    min_views_ = min_views;
}

void Triangulator::setRansacEnabled(bool enabled) {
    ransac_enabled_ = enabled;
}

void Triangulator::setRansacThreshold(float threshold_px) {
    ransac_threshold_px_ = threshold_px;
}

int Triangulator::cameraIndex(const std::string& camera_id) const {
    for (int i = 0; i < static_cast<int>(cameras_.size()); ++i) {
        if (cameras_[i].camera_id == camera_id) return i;
    }
    return -1;
}

cv::Mat Triangulator::projectionMatrix(int camera_idx) const {
    return cameras_[camera_idx].extrinsics.projectionMatrix(cameras_[camera_idx].intrinsics);
}

std::vector<Pose3D> Triangulator::triangulate(
    const std::vector<PersonTracker::TrackedPerson2D>& detections,
    double timestamp
) {
    // Group detections by global_person_id
    std::unordered_map<int, std::vector<const PersonTracker::TrackedPerson2D*>> by_person;
    for (const auto& det : detections) {
        by_person[det.global_person_id].push_back(&det);
    }

    std::vector<Pose3D> results;

    for (auto& [person_id, person_dets] : by_person) {
        Pose3D pose3d;
        pose3d.global_person_id = person_id;
        pose3d.timestamp = timestamp;

        // Find the maximum number of keypoints across all detections
        int max_kp = 0;
        for (const auto* det : person_dets) {
            max_kp = std::max(max_kp, static_cast<int>(det->pose.keypoints.size()));
        }

        // Triangulate each keypoint
        for (int kp_idx = 0; kp_idx < max_kp; ++kp_idx) {
            // Collect observations for this keypoint across cameras
            std::vector<std::pair<int, cv::Point2f>> observations;

            for (const auto* det : person_dets) {
                if (kp_idx >= static_cast<int>(det->pose.keypoints.size())) continue;

                const auto& kp = det->pose.keypoints[kp_idx];
                if (kp.conf < 0.1f) continue;  // skip very low confidence

                int cam_idx = cameraIndex(det->camera_id);
                if (cam_idx < 0) continue;

                observations.emplace_back(cam_idx, cv::Point2f(kp.x, kp.y));
            }

            if (static_cast<int>(observations.size()) < min_views_) continue;

            // Triangulate
            auto [point3d, reproj_err] = triangulateSinglePoint(observations);

            Marker3D marker;
            marker.index = kp_idx;
            if (!person_dets.empty() &&
                kp_idx < static_cast<int>(person_dets[0]->pose.keypoints.size())) {
                marker.name = person_dets[0]->pose.keypoints[kp_idx].name;
            } else {
                marker.name = "kp_" + std::to_string(kp_idx);
            }
            marker.position = point3d;
            marker.n_views = static_cast<int>(observations.size());

            // Confidence based on reprojection error (lower error = higher confidence)
            marker.confidence = std::exp(-reproj_err / 10.0f);

            pose3d.markers.push_back(marker);
        }

        if (!pose3d.markers.empty()) {
            results.push_back(std::move(pose3d));
        }
    }

    return results;
}

std::pair<Vec3f, float> Triangulator::triangulateSinglePoint(
    const std::vector<std::pair<int, cv::Point2f>>& observations
) const {
    if (static_cast<int>(observations.size()) < min_views_) {
        return {Vec3f::Zero(), 1e6f};
    }

    if (ransac_enabled_ && static_cast<int>(observations.size()) >= 3) {
        float reproj_err = 0.0f;
        Vec3f point = ransacTriangulate(observations, reproj_err);
        return {point, reproj_err};
    }

    // Direct DLT with all observations
    std::vector<std::pair<cv::Mat, cv::Point2f>> proj_and_points;
    for (const auto& [cam_idx, pt] : observations) {
        proj_and_points.emplace_back(projectionMatrix(cam_idx), pt);
    }

    Vec3f point = dlt(proj_and_points);

    // Compute mean reprojection error
    float total_err = 0.0f;
    for (const auto& [cam_idx, pt] : observations) {
        total_err += reprojectionError(point, cam_idx, pt);
    }
    float mean_err = total_err / observations.size();

    return {point, mean_err};
}

Vec3f Triangulator::dlt(
    const std::vector<std::pair<cv::Mat, cv::Point2f>>& proj_and_points
) const {
    int n = static_cast<int>(proj_and_points.size());
    cv::Mat A(2 * n, 4, CV_64F);

    for (int i = 0; i < n; ++i) {
        const cv::Mat& P = proj_and_points[i].first;
        double x = proj_and_points[i].second.x;
        double y = proj_and_points[i].second.y;

        // x * P[2,:] - P[0,:]
        A.at<double>(2 * i, 0) = x * P.at<double>(2, 0) - P.at<double>(0, 0);
        A.at<double>(2 * i, 1) = x * P.at<double>(2, 1) - P.at<double>(0, 1);
        A.at<double>(2 * i, 2) = x * P.at<double>(2, 2) - P.at<double>(0, 2);
        A.at<double>(2 * i, 3) = x * P.at<double>(2, 3) - P.at<double>(0, 3);

        // y * P[2,:] - P[1,:]
        A.at<double>(2 * i + 1, 0) = y * P.at<double>(2, 0) - P.at<double>(1, 0);
        A.at<double>(2 * i + 1, 1) = y * P.at<double>(2, 1) - P.at<double>(1, 1);
        A.at<double>(2 * i + 1, 2) = y * P.at<double>(2, 2) - P.at<double>(1, 2);
        A.at<double>(2 * i + 1, 3) = y * P.at<double>(2, 3) - P.at<double>(1, 3);
    }

    // SVD: solution is the last column of V
    cv::Mat w, u, vt;
    cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A);

    cv::Mat X = vt.row(vt.rows - 1).t();

    // Dehomogenize
    double w_val = X.at<double>(3);
    if (std::abs(w_val) < 1e-10) {
        return Vec3f::Zero();
    }

    return Vec3f(
        static_cast<float>(X.at<double>(0) / w_val),
        static_cast<float>(X.at<double>(1) / w_val),
        static_cast<float>(X.at<double>(2) / w_val)
    );
}

Vec3f Triangulator::ransacTriangulate(
    const std::vector<std::pair<int, cv::Point2f>>& observations,
    float& reprojection_error
) const {
    int n = static_cast<int>(observations.size());
    if (n < 2) {
        reprojection_error = 1e6f;
        return Vec3f::Zero();
    }

    Vec3f best_point = Vec3f::Zero();
    float best_error = 1e6f;
    int best_inliers = 0;

    std::mt19937 rng(42);  // deterministic for reproducibility
    int max_iterations = std::min(50, n * (n - 1) / 2);

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Sample 2 random observations
        int i = rng() % n;
        int j = (i + 1 + rng() % (n - 1)) % n;

        std::vector<std::pair<cv::Mat, cv::Point2f>> sample = {
            {projectionMatrix(observations[i].first), observations[i].second},
            {projectionMatrix(observations[j].first), observations[j].second}
        };

        Vec3f candidate = dlt(sample);

        // Count inliers and compute error
        int inliers = 0;
        float total_err = 0.0f;
        std::vector<int> inlier_indices;

        for (int k = 0; k < n; ++k) {
            float err = reprojectionError(candidate, observations[k].first, observations[k].second);
            if (err < ransac_threshold_px_) {
                inliers++;
                total_err += err;
                inlier_indices.push_back(k);
            }
        }

        if (inliers > best_inliers || (inliers == best_inliers && total_err < best_error)) {
            best_inliers = inliers;
            best_error = total_err;

            // Re-triangulate using all inliers
            if (inliers >= 2) {
                std::vector<std::pair<cv::Mat, cv::Point2f>> inlier_data;
                for (int idx : inlier_indices) {
                    inlier_data.emplace_back(
                        projectionMatrix(observations[idx].first),
                        observations[idx].second
                    );
                }
                best_point = dlt(inlier_data);

                // Recompute error with refined point
                best_error = 0.0f;
                for (int idx : inlier_indices) {
                    best_error += reprojectionError(best_point, observations[idx].first, observations[idx].second);
                }
            } else {
                best_point = candidate;
            }
        }
    }

    reprojection_error = (best_inliers > 0) ? best_error / best_inliers : 1e6f;
    return best_point;
}

float Triangulator::reprojectionError(
    const Vec3f& point3d,
    int camera_idx,
    const cv::Point2f& observed
) const {
    cv::Mat P = projectionMatrix(camera_idx);

    cv::Mat X = (cv::Mat_<double>(4, 1) << point3d.x(), point3d.y(), point3d.z(), 1.0);
    cv::Mat projected = P * X;

    double w = projected.at<double>(2);
    if (std::abs(w) < 1e-10) return 1e6f;

    float proj_x = static_cast<float>(projected.at<double>(0) / w);
    float proj_y = static_cast<float>(projected.at<double>(1) / w);

    float dx = proj_x - observed.x;
    float dy = proj_y - observed.y;

    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace mocap
