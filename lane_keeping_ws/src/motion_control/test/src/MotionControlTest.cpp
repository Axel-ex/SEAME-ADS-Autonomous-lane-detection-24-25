#include "MotionControlNode.hpp"
#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>

class MockPIDController : public PIDController
{
    public:
        double calculate(double error) override
        {
            last_error = error;
            return 0.42; // fixed dummy value
        }
        double last_error;
};

class MockKalmanFilter : public KalmanFilter
{
    public:
        MockKalmanFilter() : KalmanFilter(0.1, 0.4) {}
        double update(double val) override
        {
            last_input = val;
            return val; // no smoothing, identity
        }
        double last_input;
};

class MockLaneBuffer : public LaneBuffer
{
    public:
        MockLaneBuffer() : LaneBuffer(3) {}
        void addCoeffs(const std::vector<double>&,
                       const std::vector<double>&) override
        {
        }
        bool hasLeftLane() override { return true; }
        bool hasRightLane() override { return true; }
};

class MotionControlTest : ::testing::Test
{
    protected:
        MotionControlNode motion_control;
};
