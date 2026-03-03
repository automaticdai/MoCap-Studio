#include <gtest/gtest.h>
#include "pose/person_tracker.h"

using namespace mocap;

namespace {

Raw2DPose makePose(int person_id, float x, float y, float conf = 0.9f) {
    Raw2DPose pose;
    pose.person_id = person_id;
    pose.confidence = conf;
    pose.bbox = {x - 50, y - 100, 100, 200};

    // Create a few keypoints centered around (x, y)
    for (int i = 0; i < 5; ++i) {
        Keypoint2D kp;
        kp.index = i;
        kp.name = "kp_" + std::to_string(i);
        kp.x = x + (i - 2) * 10.0f;
        kp.y = y + (i - 2) * 15.0f;
        kp.conf = conf;
        pose.keypoints.push_back(kp);
    }

    return pose;
}

}  // namespace

TEST(PersonTrackerTest, FirstFrameCreatesNewTracks) {
    PersonTracker tracker;

    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> detections = {
        {"cam_0", {makePose(0, 200, 300), makePose(1, 500, 300)}}
    };

    auto results = tracker.update(detections, 0.0);
    EXPECT_EQ(results.size(), 2u);
    EXPECT_EQ(tracker.activePersonCount(), 2);

    // Global IDs should be 0 and 1
    std::vector<int> ids;
    for (const auto& r : results) ids.push_back(r.global_person_id);
    std::sort(ids.begin(), ids.end());
    EXPECT_EQ(ids[0], 0);
    EXPECT_EQ(ids[1], 1);
}

TEST(PersonTrackerTest, StableIDsAcrossFrames) {
    PersonTracker tracker;

    // Frame 1: two persons
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det1 = {
        {"cam_0", {makePose(0, 200, 300), makePose(1, 500, 300)}}
    };
    auto res1 = tracker.update(det1, 0.0);
    ASSERT_EQ(res1.size(), 2u);

    int id_left = -1, id_right = -1;
    for (const auto& r : res1) {
        if (r.pose.bbox.x < 300) id_left = r.global_person_id;
        else id_right = r.global_person_id;
    }

    // Frame 2: same persons, slightly moved
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det2 = {
        {"cam_0", {makePose(0, 205, 305), makePose(1, 495, 295)}}
    };
    auto res2 = tracker.update(det2, 1.0 / 60.0);
    ASSERT_EQ(res2.size(), 2u);

    for (const auto& r : res2) {
        if (r.pose.bbox.x < 300) {
            EXPECT_EQ(r.global_person_id, id_left);
        } else {
            EXPECT_EQ(r.global_person_id, id_right);
        }
    }
}

TEST(PersonTrackerTest, NewPersonAppearing) {
    PersonTracker tracker;

    // Frame 1: one person
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det1 = {
        {"cam_0", {makePose(0, 200, 300)}}
    };
    auto res1 = tracker.update(det1, 0.0);
    EXPECT_EQ(res1.size(), 1u);
    int first_id = res1[0].global_person_id;

    // Frame 2: two persons (one new, far away)
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det2 = {
        {"cam_0", {makePose(0, 205, 305), makePose(1, 800, 300)}}
    };
    auto res2 = tracker.update(det2, 1.0 / 60.0);
    EXPECT_EQ(res2.size(), 2u);

    bool found_original = false;
    bool found_new = false;
    for (const auto& r : res2) {
        if (r.global_person_id == first_id) found_original = true;
        if (r.global_person_id != first_id) found_new = true;
    }
    EXPECT_TRUE(found_original);
    EXPECT_TRUE(found_new);
}

TEST(PersonTrackerTest, PersonDisappearing) {
    PersonTracker tracker;
    tracker.setMaxFramesMissing(2);

    // Frame 1: one person
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det1 = {
        {"cam_0", {makePose(0, 200, 300)}}
    };
    tracker.update(det1, 0.0);
    EXPECT_EQ(tracker.activePersonCount(), 1);

    // Frames 2-4: no detections
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> empty = {
        {"cam_0", {}}
    };
    tracker.update(empty, 1.0 / 60.0);
    EXPECT_EQ(tracker.activePersonCount(), 1);  // still within tolerance

    tracker.update(empty, 2.0 / 60.0);
    EXPECT_EQ(tracker.activePersonCount(), 1);  // at limit

    tracker.update(empty, 3.0 / 60.0);
    EXPECT_EQ(tracker.activePersonCount(), 0);  // removed
}

TEST(PersonTrackerTest, Reset) {
    PersonTracker tracker;

    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det = {
        {"cam_0", {makePose(0, 200, 300)}}
    };
    tracker.update(det, 0.0);
    EXPECT_EQ(tracker.activePersonCount(), 1);

    tracker.reset();
    EXPECT_EQ(tracker.activePersonCount(), 0);

    // After reset, new IDs start from 0
    auto res = tracker.update(det, 1.0);
    EXPECT_EQ(res[0].global_person_id, 0);
}

TEST(PersonTrackerTest, MultiCameraDetections) {
    PersonTracker tracker;

    // Same person detected in two cameras at similar positions
    std::vector<std::pair<std::string, std::vector<Raw2DPose>>> det = {
        {"cam_0", {makePose(0, 200, 300)}},
        {"cam_1", {makePose(0, 210, 310)}}
    };

    auto results = tracker.update(det, 0.0);
    // The tracker should create tracks for each camera detection
    EXPECT_GE(results.size(), 1u);
    EXPECT_LE(tracker.activePersonCount(), 2);
}
