#include "pose/person_tracker.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace mocap {

std::vector<PersonTracker::TrackedPerson2D> PersonTracker::update(
    const std::vector<std::pair<std::string, std::vector<Raw2DPose>>>& camera_detections,
    double timestamp
) {
    // Flatten all detections across cameras
    struct Detection {
        Raw2DPose pose;
        std::string camera_id;
    };
    std::vector<Detection> all_detections;
    for (const auto& [cam_id, poses] : camera_detections) {
        for (const auto& pose : poses) {
            all_detections.push_back({pose, cam_id});
        }
    }

    std::vector<TrackedPerson2D> results;

    if (tracks_.empty() && all_detections.empty()) {
        return results;
    }

    // Build cost matrix [tracks x detections]
    int num_tracks = static_cast<int>(tracks_.size());
    int num_dets = static_cast<int>(all_detections.size());

    if (num_tracks > 0 && num_dets > 0) {
        std::vector<std::vector<float>> cost_matrix(num_tracks, std::vector<float>(num_dets));

        for (int t = 0; t < num_tracks; ++t) {
            for (int d = 0; d < num_dets; ++d) {
                cost_matrix[t][d] = computeCost(all_detections[d].pose, tracks_[t]);
            }
        }

        auto assignments = hungarianAssign(cost_matrix);

        std::vector<bool> track_assigned(num_tracks, false);
        std::vector<bool> det_assigned(num_dets, false);

        for (auto [track_idx, det_idx] : assignments) {
            if (cost_matrix[track_idx][det_idx] < 1.0f) {
                // Update track
                tracks_[track_idx].last_pose = all_detections[det_idx].pose;
                tracks_[track_idx].last_camera_id = all_detections[det_idx].camera_id;
                tracks_[track_idx].last_seen = timestamp;
                tracks_[track_idx].frames_since_seen = 0;

                TrackedPerson2D tp;
                tp.global_person_id = tracks_[track_idx].global_id;
                tp.pose = all_detections[det_idx].pose;
                tp.camera_id = all_detections[det_idx].camera_id;
                results.push_back(tp);

                track_assigned[track_idx] = true;
                det_assigned[det_idx] = true;
            }
        }

        // Create new tracks for unassigned detections
        for (int d = 0; d < num_dets; ++d) {
            if (!det_assigned[d]) {
                Track new_track;
                new_track.global_id = next_global_id_++;
                new_track.last_pose = all_detections[d].pose;
                new_track.last_camera_id = all_detections[d].camera_id;
                new_track.last_seen = timestamp;
                new_track.frames_since_seen = 0;
                tracks_.push_back(new_track);

                TrackedPerson2D tp;
                tp.global_person_id = new_track.global_id;
                tp.pose = all_detections[d].pose;
                tp.camera_id = all_detections[d].camera_id;
                results.push_back(tp);
            }
        }

        // Increment missing count for unassigned tracks
        for (int t = 0; t < num_tracks; ++t) {
            if (!track_assigned[t]) {
                tracks_[t].frames_since_seen++;
            }
        }
    } else if (num_dets > 0) {
        // No existing tracks — create new ones
        for (const auto& det : all_detections) {
            Track new_track;
            new_track.global_id = next_global_id_++;
            new_track.last_pose = det.pose;
            new_track.last_camera_id = det.camera_id;
            new_track.last_seen = timestamp;
            tracks_.push_back(new_track);

            TrackedPerson2D tp;
            tp.global_person_id = new_track.global_id;
            tp.pose = det.pose;
            tp.camera_id = det.camera_id;
            results.push_back(tp);
        }
    } else {
        // No detections — increment all tracks
        for (auto& track : tracks_) {
            track.frames_since_seen++;
        }
    }

    // Remove stale tracks
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
            [this](const Track& t) { return t.frames_since_seen > max_frames_missing_; }),
        tracks_.end()
    );

    return results;
}

void PersonTracker::reset() {
    tracks_.clear();
    next_global_id_ = 0;
}

int PersonTracker::activePersonCount() const {
    return static_cast<int>(tracks_.size());
}

void PersonTracker::setMaxFramesMissing(int frames) {
    max_frames_missing_ = frames;
}

void PersonTracker::setIoUThreshold(float threshold) {
    iou_threshold_ = threshold;
}

float PersonTracker::computeCost(const Raw2DPose& detection, const Track& track) const {
    float iou = detection.bbox.iou(track.last_pose.bbox);
    float kp_dist = keypointDistance(detection, track.last_pose);

    // Combined cost: lower is better
    // IoU component: 1 - IoU (0 = perfect overlap, 1 = no overlap)
    // Keypoint distance: normalized
    float iou_cost = 1.0f - iou;
    float kp_cost = std::min(kp_dist / 200.0f, 1.0f);  // normalize by ~200px

    return 0.5f * iou_cost + 0.5f * kp_cost;
}

float PersonTracker::keypointDistance(const Raw2DPose& a, const Raw2DPose& b) const {
    if (a.keypoints.empty() || b.keypoints.empty()) return 1000.0f;

    int count = 0;
    float sum_dist = 0.0f;

    int n = std::min(static_cast<int>(a.keypoints.size()),
                     static_cast<int>(b.keypoints.size()));

    for (int i = 0; i < n; ++i) {
        if (a.keypoints[i].conf > 0.1f && b.keypoints[i].conf > 0.1f) {
            float dx = a.keypoints[i].x - b.keypoints[i].x;
            float dy = a.keypoints[i].y - b.keypoints[i].y;
            sum_dist += std::sqrt(dx * dx + dy * dy);
            count++;
        }
    }

    return (count > 0) ? sum_dist / count : 1000.0f;
}

std::vector<std::pair<int, int>> PersonTracker::hungarianAssign(
    const std::vector<std::vector<float>>& cost_matrix
) const {
    // Simplified Hungarian algorithm for assignment
    // For production use, a full Kuhn-Munkres implementation is preferred
    int rows = static_cast<int>(cost_matrix.size());
    int cols = cost_matrix.empty() ? 0 : static_cast<int>(cost_matrix[0].size());

    if (rows == 0 || cols == 0) return {};

    int n = std::max(rows, cols);

    // Pad to square matrix
    std::vector<std::vector<float>> padded(n, std::vector<float>(n, 1e9f));
    for (int i = 0; i < rows; ++i)
        for (int j = 0; j < cols; ++j)
            padded[i][j] = cost_matrix[i][j];

    // Step 1: Subtract row minimums
    for (int i = 0; i < n; ++i) {
        float row_min = *std::min_element(padded[i].begin(), padded[i].end());
        for (int j = 0; j < n; ++j) padded[i][j] -= row_min;
    }

    // Step 2: Subtract column minimums
    for (int j = 0; j < n; ++j) {
        float col_min = padded[0][j];
        for (int i = 1; i < n; ++i) col_min = std::min(col_min, padded[i][j]);
        for (int i = 0; i < n; ++i) padded[i][j] -= col_min;
    }

    // Greedy assignment on reduced cost matrix
    std::vector<int> row_assign(n, -1);
    std::vector<int> col_assign(n, -1);

    for (int iter = 0; iter < n * 2; ++iter) {
        // Find the row with fewest zeros
        int best_row = -1;
        int min_zeros = n + 1;

        for (int i = 0; i < n; ++i) {
            if (row_assign[i] != -1) continue;
            int zero_count = 0;
            for (int j = 0; j < n; ++j) {
                if (col_assign[j] == -1 && std::abs(padded[i][j]) < 1e-6f) {
                    zero_count++;
                }
            }
            if (zero_count > 0 && zero_count < min_zeros) {
                min_zeros = zero_count;
                best_row = i;
            }
        }

        if (best_row == -1) break;

        // Assign to the column with the smallest original cost
        int best_col = -1;
        float best_cost = std::numeric_limits<float>::max();
        for (int j = 0; j < n; ++j) {
            if (col_assign[j] == -1 && std::abs(padded[best_row][j]) < 1e-6f) {
                float orig_cost = (best_row < rows && j < cols)
                    ? cost_matrix[best_row][j] : 1e9f;
                if (orig_cost < best_cost) {
                    best_cost = orig_cost;
                    best_col = j;
                }
            }
        }

        if (best_col != -1) {
            row_assign[best_row] = best_col;
            col_assign[best_col] = best_row;
        }
    }

    // Collect valid assignments (only original rows/cols)
    std::vector<std::pair<int, int>> result;
    for (int i = 0; i < rows; ++i) {
        if (row_assign[i] >= 0 && row_assign[i] < cols) {
            result.emplace_back(i, row_assign[i]);
        }
    }

    return result;
}

}  // namespace mocap
