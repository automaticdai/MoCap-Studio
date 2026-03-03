#include <gtest/gtest.h>
#include "core/types.h"

using namespace mocap;

TEST(Keypoint2DTest, DefaultConstruction) {
    Keypoint2D kp;
    EXPECT_EQ(kp.index, 0);
    EXPECT_FLOAT_EQ(kp.x, 0.0f);
    EXPECT_FLOAT_EQ(kp.y, 0.0f);
    EXPECT_FLOAT_EQ(kp.conf, 0.0f);
    EXPECT_TRUE(kp.name.empty());
}

TEST(Keypoint2DTest, FieldAccess) {
    Keypoint2D kp;
    kp.name = "left_shoulder";
    kp.index = 5;
    kp.x = 320.5f;
    kp.y = 240.3f;
    kp.conf = 0.95f;

    EXPECT_EQ(kp.name, "left_shoulder");
    EXPECT_EQ(kp.index, 5);
    EXPECT_FLOAT_EQ(kp.x, 320.5f);
    EXPECT_FLOAT_EQ(kp.y, 240.3f);
    EXPECT_FLOAT_EQ(kp.conf, 0.95f);
}

TEST(RectTest, Area) {
    Rect r{10.0f, 20.0f, 100.0f, 50.0f};
    EXPECT_FLOAT_EQ(r.area(), 5000.0f);
}

TEST(RectTest, IoUIdentical) {
    Rect r{0.0f, 0.0f, 100.0f, 100.0f};
    EXPECT_FLOAT_EQ(r.iou(r), 1.0f);
}

TEST(RectTest, IoUNoOverlap) {
    Rect a{0.0f, 0.0f, 10.0f, 10.0f};
    Rect b{100.0f, 100.0f, 10.0f, 10.0f};
    EXPECT_FLOAT_EQ(a.iou(b), 0.0f);
}

TEST(RectTest, IoUPartialOverlap) {
    Rect a{0.0f, 0.0f, 10.0f, 10.0f};
    Rect b{5.0f, 5.0f, 10.0f, 10.0f};
    // Intersection: 5x5=25, Union: 100+100-25=175
    EXPECT_NEAR(a.iou(b), 25.0f / 175.0f, 1e-5f);
}

TEST(Raw2DPoseTest, Construction) {
    Raw2DPose pose;
    EXPECT_EQ(pose.person_id, 0);
    EXPECT_TRUE(pose.keypoints.empty());
    EXPECT_FLOAT_EQ(pose.confidence, 0.0f);
}

TEST(Marker3DTest, DefaultPosition) {
    Marker3D m;
    EXPECT_EQ(m.position, Vec3f::Zero());
    EXPECT_FLOAT_EQ(m.confidence, 0.0f);
    EXPECT_EQ(m.n_views, 0);
}

TEST(Pose3DTest, Construction) {
    Pose3D pose;
    pose.global_person_id = 42;
    pose.timestamp = 1.5;

    Marker3D m;
    m.name = "nose";
    m.index = 0;
    m.position = Vec3f(1.0f, 2.0f, 3.0f);
    m.confidence = 0.9f;
    m.n_views = 3;
    pose.markers.push_back(m);

    EXPECT_EQ(pose.global_person_id, 42);
    EXPECT_DOUBLE_EQ(pose.timestamp, 1.5);
    EXPECT_EQ(pose.markers.size(), 1u);
    EXPECT_EQ(pose.markers[0].name, "nose");
    EXPECT_FLOAT_EQ(pose.markers[0].position.x(), 1.0f);
}

TEST(JointRotationTest, DefaultIdentity) {
    JointRotation jr;
    EXPECT_EQ(jr.rotation.w(), 1.0f);
    EXPECT_EQ(jr.rotation.x(), 0.0f);
    EXPECT_EQ(jr.rotation.y(), 0.0f);
    EXPECT_EQ(jr.rotation.z(), 0.0f);
    EXPECT_EQ(jr.euler_xyz, Vec3f::Zero());
}

TEST(SkeletonPoseTest, Construction) {
    SkeletonPose sp;
    EXPECT_EQ(sp.global_person_id, 0);
    EXPECT_DOUBLE_EQ(sp.timestamp, 0.0);
    EXPECT_EQ(sp.root_position, Vec3f::Zero());
    EXPECT_TRUE(sp.joint_rotations.empty());
}

TEST(CapturedFrameTest, Construction) {
    CapturedFrame cf;
    EXPECT_TRUE(cf.camera_id.empty());
    EXPECT_DOUBLE_EQ(cf.timestamp, 0.0);
    EXPECT_EQ(cf.frame_number, 0);
    EXPECT_TRUE(cf.image.empty());
}

TEST(FrameSetTest, Construction) {
    FrameSet fs;
    EXPECT_DOUBLE_EQ(fs.timestamp, 0.0);
    EXPECT_TRUE(fs.frames.empty());
}

TEST(FramePacketTest, Magic) {
    FramePacket pkt;
    EXPECT_EQ(pkt.magic, 0x4D435030u);
    EXPECT_TRUE(pkt.isValid());
    EXPECT_EQ(pkt.version, 1);
}

TEST(FramePacketTest, InvalidMagic) {
    FramePacket pkt;
    pkt.magic = 0x00000000;
    EXPECT_FALSE(pkt.isValid());
}
