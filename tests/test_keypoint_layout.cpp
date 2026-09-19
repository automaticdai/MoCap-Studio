#include <gtest/gtest.h>
#include "pose/keypoint_layout.h"

TEST(KeypointLayout, MapsSidesFeetAndDerivedJoints) {
    std::vector<mocap::Keypoint2D> source(133);
    for (int i = 0; i < 133; ++i) {
        source[i].x = float(i);
        source[i].y = float(i * 2);
        source[i].conf = 0.9f;
    }
    source[6].conf = 0.2f;
    source[12].conf = 0.3f;
    const auto result = mocap::wholebodyToBody25(source);
    ASSERT_EQ(result.size(), 25u);
    EXPECT_FLOAT_EQ(result[2].x, 6);   // Right shoulder.
    EXPECT_FLOAT_EQ(result[5].x, 5);   // Left shoulder.
    EXPECT_FLOAT_EQ(result[19].x, 17); // Left big toe.
    EXPECT_FLOAT_EQ(result[24].x, 22); // Right heel.
    EXPECT_FLOAT_EQ(result[1].x, 5.5f);
    EXPECT_FLOAT_EQ(result[1].y, 11);
    EXPECT_FLOAT_EQ(result[1].conf, 0.2f);
    EXPECT_FLOAT_EQ(result[8].x, 11.5f);
    EXPECT_FLOAT_EQ(result[8].conf, 0.3f);
    for (int i = 0; i < 25; ++i) EXPECT_EQ(result[i].index, i);
}

TEST(KeypointLayout, RejectsOtherLayouts) {
    EXPECT_THROW(mocap::wholebodyToBody25(std::vector<mocap::Keypoint2D>(17)),
                 std::invalid_argument);
}
