#include <gtest/gtest.h>
#include "calibration_test_data.h"
#include "triangulation/triangulator.h"
#include <limits>

TEST(StereoCalibration, RecoversMetricGeometryWithDifferentLensesAndResolutions) {
    SyntheticRig rig;
    const auto result = mocap::calibrateStereo(rig.left, rig.right, rig.first, rig.second, rig.board);
    ASSERT_TRUE(result.success) << result.error;
    EXPECT_LT(result.rms, 0.01);
    EXPECT_LT(cv::norm(result.extrinsics.rotation - rig.relative.rotation), 1e-4);
    EXPECT_LT(cv::norm(result.extrinsics.translation - rig.relative.translation), 1e-4);
}

TEST(StereoCalibration, RejectsUnpairedInvalidAndRepeatedObservations) {
    SyntheticRig rig;
    auto invalid = rig.right;
    invalid.pop_back();
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, invalid, rig.first, rig.second, rig.board).success);
    invalid = rig.right; invalid[3].pop_back();
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, invalid, rig.first, rig.second, rig.board).success);
    invalid = rig.right; invalid[3][0].x = std::numeric_limits<float>::quiet_NaN();
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, invalid, rig.first, rig.second, rig.board).success);
    invalid.assign(14, rig.right.front());
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, invalid, rig.first, rig.second, rig.board).success);
    auto board = rig.board; board.square_size_m = 0;
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, rig.right, rig.first, rig.second, board).success);
    auto intrinsics = rig.first; intrinsics.fx = 0;
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, rig.right, intrinsics, rig.second, rig.board).success);
}

TEST(StereoCalibration, RejectsMismatchedBoardPosesAndZeroBaseline) {
    SyntheticRig rig;
    auto mismatched = rig.right;
    std::reverse(mismatched.begin(), mismatched.end());
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, mismatched, rig.first, rig.second, rig.board).success);
    EXPECT_FALSE(mocap::calibrateStereo(rig.left, rig.left, rig.first, rig.first, rig.board).success);
}

TEST(StereoCalibration, CalibratedViewsAssociateAndReconstructDistortedObservations) {
    SyntheticRig rig;
    auto result = mocap::calibrateStereo(rig.left, rig.right, rig.first, rig.second, rig.board);
    ASSERT_TRUE(result.success) << result.error;
    mocap::Triangulator triangulator;
    triangulator.setCameras({{rig.first, {}, "left"}, {rig.second, result.extrinsics, "right"}});
    std::vector<mocap::PersonTracker::TrackedPerson2D> detections(2);
    for (int i = 0; i < 2; ++i) {
        detections[i].camera_id = i ? "right" : "left";
        detections[i].global_person_id = i + 10;
        const auto& points = i ? rig.right[0] : rig.left[0];
        for (size_t k = 0; k < points.size(); ++k) {
            mocap::Keypoint2D keypoint;
            keypoint.index = k; keypoint.x = points[k].x; keypoint.y = points[k].y; keypoint.conf = 1;
            detections[i].pose.keypoints.push_back(keypoint);
        }
    }
    auto associated = triangulator.associateAcrossCameras(detections);
    ASSERT_EQ(associated[0].global_person_id, associated[1].global_person_id);
    auto poses = triangulator.triangulate(associated, 1);
    ASSERT_EQ(poses.size(), 1u);
    ASSERT_EQ(poses[0].markers.size(), rig.left[0].size());
    const auto first = poses[0].markers[0].position;
    EXPECT_NEAR(first.x(), -0.18, 0.001);
    EXPECT_NEAR(first.y(), -0.12, 0.001);
    EXPECT_NEAR(first.z(), 0.8, 0.001);
    auto duplicate = detections[1];
    duplicate.global_person_id = 12;
    detections.push_back(duplicate);
    associated = triangulator.associateAcrossCameras(detections);
    EXPECT_NE(associated[1].global_person_id, associated[2].global_person_id);
    detections.pop_back();
    // A different person's keypoints should not be joined solely by camera count.
    for (auto& point : detections[1].pose.keypoints) point.y += 200;
    associated = triangulator.associateAcrossCameras(detections);
    EXPECT_NE(associated[0].global_person_id, associated[1].global_person_id);
    EXPECT_TRUE(triangulator.triangulate(associated, 2).empty());
}
