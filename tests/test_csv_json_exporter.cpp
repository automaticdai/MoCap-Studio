#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>
#include "storage/exporters/csv_exporter.h"
#include "storage/exporters/json_exporter.h"

using namespace mocap;
namespace fs = std::filesystem;

class ExporterTest : public ::testing::Test {
protected:
    std::string temp_dir;

    std::vector<std::pair<double, std::vector<Raw2DPose>>> raw2d_data;
    std::vector<std::pair<double, std::vector<Pose3D>>> pose3d_data;
    std::vector<std::pair<double, std::vector<SkeletonPose>>> skeleton_data;

    void SetUp() override {
        temp_dir = (fs::temp_directory_path() / "mocap_test_exporters").string();
        fs::create_directories(temp_dir);

        // Raw2D data
        Raw2DPose rp;
        rp.person_id = 0;
        rp.confidence = 0.9f;
        rp.bbox = {10, 20, 100, 200};
        Keypoint2D kp;
        kp.name = "nose"; kp.index = 0; kp.x = 50; kp.y = 30; kp.conf = 0.95f;
        rp.keypoints.push_back(kp);
        kp.name = "neck"; kp.index = 1; kp.x = 50; kp.y = 60; kp.conf = 0.85f;
        rp.keypoints.push_back(kp);
        raw2d_data.push_back({0.0, {rp}});
        raw2d_data.push_back({1.0 / 60.0, {rp}});

        // Pose3D data
        Pose3D p3;
        p3.global_person_id = 0;
        Marker3D m;
        m.name = "nose"; m.index = 0; m.position = Vec3f(1, 2, 3); m.confidence = 0.9f; m.n_views = 3;
        p3.markers.push_back(m);
        pose3d_data.push_back({0.0, {p3}});

        // Skeleton data
        SkeletonPose sp;
        sp.global_person_id = 0;
        sp.root_position = Vec3f(0, 1.5f, 0);
        sp.root_rotation = Quaternion::Identity();
        JointRotation jr;
        jr.joint_name = "neck"; jr.joint_index = 1;
        jr.rotation = Quaternion::Identity();
        jr.euler_xyz = Vec3f::Zero();
        sp.joint_rotations.push_back(jr);
        skeleton_data.push_back({0.0, {sp}});
    }

    void TearDown() override {
        fs::remove_all(temp_dir);
    }
};

// --- CSV Tests ---

TEST_F(ExporterTest, CsvRaw2D) {
    std::string path = (fs::path(temp_dir) / "raw2d.csv").string();
    CsvExporter::exportRaw2D(path, raw2d_data);

    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());

    std::string header;
    std::getline(f, header);
    EXPECT_TRUE(header.find("timestamp") != std::string::npos);
    EXPECT_TRUE(header.find("keypoint_name") != std::string::npos);

    // Should have 4 data lines (2 frames x 1 pose x 2 keypoints)
    int lines = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (!line.empty()) lines++;
    }
    EXPECT_EQ(lines, 4);
}

TEST_F(ExporterTest, CsvPose3D) {
    std::string path = (fs::path(temp_dir) / "pose3d.csv").string();
    CsvExporter::exportPose3D(path, pose3d_data);

    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());

    std::string header;
    std::getline(f, header);
    EXPECT_TRUE(header.find("marker_name") != std::string::npos);
    EXPECT_TRUE(header.find("x,y,z") != std::string::npos);

    std::string line;
    std::getline(f, line);
    EXPECT_TRUE(line.find("nose") != std::string::npos);
}

TEST_F(ExporterTest, CsvSkeleton) {
    std::string path = (fs::path(temp_dir) / "skeleton.csv").string();
    CsvExporter::exportSkeleton(path, skeleton_data);

    std::ifstream f(path);
    ASSERT_TRUE(f.is_open());

    std::string header;
    std::getline(f, header);
    EXPECT_TRUE(header.find("joint_name") != std::string::npos);
    EXPECT_TRUE(header.find("euler_x") != std::string::npos);
}

// --- JSON Tests ---

TEST_F(ExporterTest, JsonRaw2D) {
    std::string path = (fs::path(temp_dir) / "raw2d.json").string();
    JsonExporter::exportRaw2D(path, raw2d_data);

    std::ifstream f(path);
    nlohmann::json j;
    f >> j;

    EXPECT_EQ(j["layer"], "raw_2d");
    EXPECT_EQ(j["frame_count"], 2);
    ASSERT_EQ(j["frames"].size(), 2u);
    EXPECT_DOUBLE_EQ(j["frames"][0]["timestamp"].get<double>(), 0.0);

    auto& poses = j["frames"][0]["poses"];
    ASSERT_EQ(poses.size(), 1u);
    EXPECT_EQ(poses[0]["person_id"], 0);
    ASSERT_EQ(poses[0]["keypoints"].size(), 2u);
    EXPECT_EQ(poses[0]["keypoints"][0]["name"], "nose");
}

TEST_F(ExporterTest, JsonPose3D) {
    std::string path = (fs::path(temp_dir) / "pose3d.json").string();
    JsonExporter::exportPose3D(path, pose3d_data);

    std::ifstream f(path);
    nlohmann::json j;
    f >> j;

    EXPECT_EQ(j["layer"], "pose_3d");
    auto& markers = j["frames"][0]["poses"][0]["markers"];
    ASSERT_EQ(markers.size(), 1u);
    EXPECT_EQ(markers[0]["name"], "nose");

    auto pos = markers[0]["position"];
    EXPECT_FLOAT_EQ(pos[0].get<float>(), 1.0f);
    EXPECT_FLOAT_EQ(pos[1].get<float>(), 2.0f);
    EXPECT_FLOAT_EQ(pos[2].get<float>(), 3.0f);
}

TEST_F(ExporterTest, JsonSkeleton) {
    std::string path = (fs::path(temp_dir) / "skeleton.json").string();
    JsonExporter::exportSkeleton(path, skeleton_data);

    std::ifstream f(path);
    nlohmann::json j;
    f >> j;

    EXPECT_EQ(j["layer"], "skeleton");
    auto& pose = j["frames"][0]["poses"][0];
    EXPECT_EQ(pose["global_person_id"], 0);

    auto root_pos = pose["root_position"];
    EXPECT_FLOAT_EQ(root_pos[1].get<float>(), 1.5f);

    ASSERT_EQ(pose["joint_rotations"].size(), 1u);
    EXPECT_EQ(pose["joint_rotations"][0]["joint_name"], "neck");
}
