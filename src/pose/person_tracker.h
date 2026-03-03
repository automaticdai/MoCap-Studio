#pragma once

#include <vector>
#include <string>
#include "core/types.h"

namespace mocap {

class PersonTracker {
public:
    struct TrackedPerson2D {
        int global_person_id = 0;
        Raw2DPose pose;
        std::string camera_id;
    };

    // Process detections from all cameras for one time step
    std::vector<TrackedPerson2D> update(
        const std::vector<std::pair<std::string, std::vector<Raw2DPose>>>& camera_detections,
        double timestamp
    );

    void reset();
    int activePersonCount() const;

    void setMaxFramesMissing(int frames);
    void setIoUThreshold(float threshold);

private:
    struct Track {
        int global_id = 0;
        Raw2DPose last_pose;
        std::string last_camera_id;
        double last_seen = 0.0;
        int frames_since_seen = 0;
    };

    std::vector<Track> tracks_;
    int next_global_id_ = 0;
    int max_frames_missing_ = 30;
    float iou_threshold_ = 0.3f;

    // Compute cost between a detection and an existing track
    float computeCost(const Raw2DPose& detection, const Track& track) const;

    // Compute keypoint distance between two poses
    float keypointDistance(const Raw2DPose& a, const Raw2DPose& b) const;

    // Hungarian algorithm for optimal assignment
    std::vector<std::pair<int, int>> hungarianAssign(
        const std::vector<std::vector<float>>& cost_matrix
    ) const;
};

}  // namespace mocap
