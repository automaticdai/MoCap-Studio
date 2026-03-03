#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include "storage/exporters/bvh_exporter.h"

using namespace mocap;
namespace fs = std::filesystem;

class BvhExporterTest : public ::testing::Test {
protected:
    std::string temp_dir;
    SkeletonDefinition skeleton;

    void SetUp() override {
        temp_dir = (fs::temp_directory_path() / "mocap_test_bvh").string();
        fs::create_directories(temp_dir);
        skeleton = SkeletonDefinition::defaultBody25();
    }

    void TearDown() override {
        fs::remove_all(temp_dir);
    }

    std::vector<std::pair<double, std::vector<SkeletonPose>>> makeTestFrames(int count) {
        std::vector<std::pair<double, std::vector<SkeletonPose>>> frames;

        for (int i = 0; i < count; ++i) {
            SkeletonPose sp;
            sp.global_person_id = 0;
            sp.timestamp = i / 60.0;
            sp.root_position = Vec3f(0, 1.5f, 0);
            sp.root_rotation = Quaternion::Identity();

            for (int j = 0; j < skeleton.jointCount(); ++j) {
                JointRotation jr;
                jr.joint_name = skeleton.joint(j).name;
                jr.joint_index = j;
                jr.rotation = Quaternion::Identity();
                jr.euler_xyz = Vec3f::Zero();
                sp.joint_rotations.push_back(jr);
            }

            frames.push_back({sp.timestamp, {sp}});
        }
        return frames;
    }
};

TEST_F(BvhExporterTest, HierarchySection) {
    std::string path = (fs::path(temp_dir) / "test.bvh").string();
    auto frames = makeTestFrames(1);

    BvhExporter::exportSkeleton(path, skeleton, frames, 60.0, 0);

    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());

    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());

    // Should have HIERARCHY header
    EXPECT_TRUE(content.find("HIERARCHY") != std::string::npos);

    // Should have ROOT joint (neck is root in BODY_25)
    EXPECT_TRUE(content.find("ROOT neck") != std::string::npos);

    // Should have JOINT entries
    EXPECT_TRUE(content.find("JOINT nose") != std::string::npos);
    EXPECT_TRUE(content.find("JOINT right_shoulder") != std::string::npos);
    EXPECT_TRUE(content.find("JOINT left_shoulder") != std::string::npos);

    // Root should have 6 channels
    EXPECT_TRUE(content.find("CHANNELS 6") != std::string::npos);

    // Other joints should have 3 channels
    EXPECT_TRUE(content.find("CHANNELS 3") != std::string::npos);

    // Should have End Site for leaf joints
    EXPECT_TRUE(content.find("End Site") != std::string::npos);
}

TEST_F(BvhExporterTest, MotionSection) {
    std::string path = (fs::path(temp_dir) / "test.bvh").string();
    auto frames = makeTestFrames(10);

    BvhExporter::exportSkeleton(path, skeleton, frames, 60.0, 0);

    std::ifstream f(path);
    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());

    EXPECT_TRUE(content.find("MOTION") != std::string::npos);
    EXPECT_TRUE(content.find("Frames: 10") != std::string::npos);
    EXPECT_TRUE(content.find("Frame Time:") != std::string::npos);
}

TEST_F(BvhExporterTest, FrameCount) {
    std::string path = (fs::path(temp_dir) / "test.bvh").string();
    auto frames = makeTestFrames(5);

    BvhExporter::exportSkeleton(path, skeleton, frames, 30.0, 0);

    std::ifstream f(path);
    std::string line;
    bool in_motion = false;
    int data_lines = 0;

    while (std::getline(f, line)) {
        if (line.find("Frame Time:") != std::string::npos) {
            in_motion = true;
            continue;
        }
        if (in_motion && !line.empty()) {
            data_lines++;
        }
    }

    EXPECT_EQ(data_lines, 5);
}

TEST_F(BvhExporterTest, RootPosition) {
    std::string path = (fs::path(temp_dir) / "test.bvh").string();
    auto frames = makeTestFrames(1);
    // Root at (0, 1.5, 0) metres = (0, 150, 0) cm
    frames[0].second[0].root_position = Vec3f(0, 1.5f, 0);

    BvhExporter::exportSkeleton(path, skeleton, frames, 60.0, 0);

    std::ifstream f(path);
    std::string line;
    bool in_motion = false;

    while (std::getline(f, line)) {
        if (line.find("Frame Time:") != std::string::npos) {
            in_motion = true;
            continue;
        }
        if (in_motion && !line.empty()) {
            // First 3 values should be root position in cm
            std::istringstream iss(line);
            float px, py, pz;
            iss >> px >> py >> pz;
            EXPECT_NEAR(px, 0.0f, 0.01f);
            EXPECT_NEAR(py, 150.0f, 0.01f);
            EXPECT_NEAR(pz, 0.0f, 0.01f);
            break;
        }
    }
}

TEST_F(BvhExporterTest, EmptyFrames) {
    std::string path = (fs::path(temp_dir) / "empty.bvh").string();
    std::vector<std::pair<double, std::vector<SkeletonPose>>> frames;

    BvhExporter::exportSkeleton(path, skeleton, frames, 60.0, 0);

    std::ifstream f(path);
    std::string content((std::istreambuf_iterator<char>(f)),
                        std::istreambuf_iterator<char>());

    EXPECT_TRUE(content.find("Frames: 0") != std::string::npos);
}
