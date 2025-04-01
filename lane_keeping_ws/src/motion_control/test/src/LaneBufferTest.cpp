#include "LaneBuffer.hpp"
#include <gtest/gtest.h>

class LaneBufferTest : public ::testing::Test
{
    protected:
        LaneBufferTest() : lane_buffer(3) {}

        LaneBuffer lane_buffer;
};

TEST_F(LaneBufferTest, TestInitialization)
{
    EXPECT_FALSE(lane_buffer.hasLeftLane());
    EXPECT_FALSE(lane_buffer.hasRightLane());
}

TEST_F(LaneBufferTest, TestAddCoeffs)
{
    std::vector<double> left_coefs = {1.0, 2.0, 3.0};
    std::vector<double> right_coefs = {5.0, 6.0, 7.0};

    lane_buffer.addCoeffs(left_coefs, right_coefs);

    EXPECT_TRUE(lane_buffer.hasLeftLane());
    EXPECT_TRUE(lane_buffer.hasRightLane());

    EXPECT_EQ(lane_buffer.getLastLeft(), left_coefs);
    EXPECT_EQ(lane_buffer.getLastRight(), right_coefs);
}

TEST_F(LaneBufferTest, TestBufferLimit)
{
    std::vector<double> left1 = {1.0, 1.1, 1.2};
    std::vector<double> left2 = {2.0, 2.1, 2.2};
    std::vector<double> left3 = {3.0, 3.1, 3.2};
    std::vector<double> left4 = {4.0, 4.1, 4.2};

    lane_buffer.addCoeffs(left1, left1);
    lane_buffer.addCoeffs(left2, left2);
    lane_buffer.addCoeffs(left3, left3);
    lane_buffer.addCoeffs(left4, left4);

    EXPECT_EQ(lane_buffer.getLastLeft(), left4);
}

TEST_F(LaneBufferTest, TestGetLastCoefficients)
{
    std::vector<double> left_coefs = {10.0, 20.0, 30.0};
    std::vector<double> right_coefs = {50.0, 60.0, 70.0};

    lane_buffer.addCoeffs(left_coefs, right_coefs);

    EXPECT_EQ(lane_buffer.getLastLeft(), left_coefs);
    EXPECT_EQ(lane_buffer.getLastRight(), right_coefs);
}
