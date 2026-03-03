#include <gtest/gtest.h>
#include <filesystem>
#include "storage/binary_io.h"

using namespace mocap;
namespace fs = std::filesystem;

class BinaryIOTest : public ::testing::Test {
protected:
    std::string temp_dir;

    void SetUp() override {
        temp_dir = (fs::temp_directory_path() / "mocap_test_binary_io").string();
        fs::create_directories(temp_dir);
    }

    void TearDown() override {
        fs::remove_all(temp_dir);
    }
};

TEST_F(BinaryIOTest, RoundTripRaw2D) {
    std::string path = (fs::path(temp_dir) / "raw_2d.bin").string();

    // Create test data
    std::vector<std::pair<double, std::vector<Raw2DPose>>> frames;

    Raw2DPose pose1;
    pose1.person_id = 0;
    pose1.confidence = 0.95f;
    pose1.bbox = {10, 20, 100, 200};

    Keypoint2D kp;
    kp.name = "nose";
    kp.index = 0;
    kp.x = 50.5f;
    kp.y = 30.2f;
    kp.conf = 0.9f;
    pose1.keypoints.push_back(kp);

    kp.name = "neck";
    kp.index = 1;
    kp.x = 50.0f;
    kp.y = 60.0f;
    kp.conf = 0.85f;
    pose1.keypoints.push_back(kp);

    frames.push_back({0.0, {pose1}});
    frames.push_back({1.0 / 60.0, {pose1}});

    // Write
    BinaryIO::writeRaw2D(path, frames);

    // Read back
    auto loaded = BinaryIO::readRaw2D(path);

    ASSERT_EQ(loaded.size(), 2u);
    EXPECT_DOUBLE_EQ(loaded[0].first, 0.0);
    ASSERT_EQ(loaded[0].second.size(), 1u);

    const auto& lp = loaded[0].second[0];
    EXPECT_EQ(lp.person_id, 0);
    EXPECT_FLOAT_EQ(lp.confidence, 0.95f);
    EXPECT_FLOAT_EQ(lp.bbox.x, 10.0f);
    EXPECT_FLOAT_EQ(lp.bbox.width, 100.0f);

    ASSERT_EQ(lp.keypoints.size(), 2u);
    EXPECT_EQ(lp.keypoints[0].name, "nose");
    EXPECT_FLOAT_EQ(lp.keypoints[0].x, 50.5f);
    EXPECT_FLOAT_EQ(lp.keypoints[0].conf, 0.9f);
}

TEST_F(BinaryIOTest, RoundTripPose3D) {
    std::string path = (fs::path(temp_dir) / "pose_3d.bin").string();

    std::vector<std::pair<double, std::vector<Pose3D>>> frames;

    Pose3D pose;
    pose.global_person_id = 42;

    Marker3D m;
    m.name = "nose";
    m.index = 0;
    m.position = Vec3f(1.0f, 2.0f, 3.0f);
    m.confidence = 0.9f;
    m.n_views = 3;
    pose.markers.push_back(m);

    m.name = "neck";
    m.index = 1;
    m.position = Vec3f(1.0f, 1.5f, 3.0f);
    m.confidence = 0.85f;
    m.n_views = 2;
    pose.markers.push_back(m);

    frames.push_back({0.5, {pose}});

    BinaryIO::writePose3D(path, frames);
    auto loaded = BinaryIO::readPose3D(path);

    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_DOUBLE_EQ(loaded[0].first, 0.5);
    ASSERT_EQ(loaded[0].second.size(), 1u);

    const auto& lp = loaded[0].second[0];
    EXPECT_EQ(lp.global_person_id, 42);
    ASSERT_EQ(lp.markers.size(), 2u);
    EXPECT_EQ(lp.markers[0].name, "nose");
    EXPECT_FLOAT_EQ(lp.markers[0].position.x(), 1.0f);
    EXPECT_FLOAT_EQ(lp.markers[0].position.y(), 2.0f);
    EXPECT_FLOAT_EQ(lp.markers[0].position.z(), 3.0f);
    EXPECT_EQ(lp.markers[0].n_views, 3);
}

TEST_F(BinaryIOTest, RoundTripSkeleton) {
    std::string path = (fs::path(temp_dir) / "skeleton.bin").string();

    std::vector<std::pair<double, std::vector<SkeletonPose>>> frames;

    SkeletonPose skel;
    skel.global_person_id = 7;
    skel.root_position = Vec3f(0.0f, 1.5f, 0.0f);
    skel.root_rotation = Quaternion(0.707f, 0.0f, 0.707f, 0.0f);

    JointRotation jr;
    jr.joint_name = "right_shoulder";
    jr.joint_index = 2;
    jr.rotation = Quaternion(0.9f, 0.1f, 0.2f, 0.3f).normalized();
    jr.euler_xyz = Vec3f(10.0f, 20.0f, 30.0f);
    skel.joint_rotations.push_back(jr);

    frames.push_back({1.0, {skel}});

    BinaryIO::writeSkeleton(path, frames);
    auto loaded = BinaryIO::readSkeleton(path);

    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_DOUBLE_EQ(loaded[0].first, 1.0);
    ASSERT_EQ(loaded[0].second.size(), 1u);

    const auto& ls = loaded[0].second[0];
    EXPECT_EQ(ls.global_person_id, 7);
    EXPECT_NEAR(ls.root_position.y(), 1.5f, 1e-5f);

    ASSERT_EQ(ls.joint_rotations.size(), 1u);
    EXPECT_EQ(ls.joint_rotations[0].joint_name, "right_shoulder");
    EXPECT_EQ(ls.joint_rotations[0].joint_index, 2);
    EXPECT_NEAR(ls.joint_rotations[0].euler_xyz.x(), 10.0f, 1e-4f);
}

TEST_F(BinaryIOTest, MultipleFramesMultiplePersons) {
    std::string path = (fs::path(temp_dir) / "multi.bin").string();

    std::vector<std::pair<double, std::vector<Pose3D>>> frames;

    for (int f = 0; f < 10; ++f) {
        std::vector<Pose3D> poses;
        for (int p = 0; p < 3; ++p) {
            Pose3D pose;
            pose.global_person_id = p;

            Marker3D m;
            m.name = "m0";
            m.index = 0;
            m.position = Vec3f(static_cast<float>(p), static_cast<float>(f), 0.0f);
            m.confidence = 0.9f;
            m.n_views = 2;
            pose.markers.push_back(m);

            poses.push_back(pose);
        }
        frames.emplace_back(f / 60.0, std::move(poses));
    }

    BinaryIO::writePose3D(path, frames);
    auto loaded = BinaryIO::readPose3D(path);

    ASSERT_EQ(loaded.size(), 10u);
    for (int f = 0; f < 10; ++f) {
        ASSERT_EQ(loaded[f].second.size(), 3u);
        for (int p = 0; p < 3; ++p) {
            EXPECT_EQ(loaded[f].second[p].global_person_id, p);
            EXPECT_FLOAT_EQ(loaded[f].second[p].markers[0].position.x(),
                           static_cast<float>(p));
        }
    }
}

TEST_F(BinaryIOTest, EmptyFrames) {
    std::string path = (fs::path(temp_dir) / "empty.bin").string();

    std::vector<std::pair<double, std::vector<Pose3D>>> frames;
    frames.push_back({0.0, {}});

    BinaryIO::writePose3D(path, frames);
    auto loaded = BinaryIO::readPose3D(path);

    ASSERT_EQ(loaded.size(), 1u);
    EXPECT_TRUE(loaded[0].second.empty());
}
