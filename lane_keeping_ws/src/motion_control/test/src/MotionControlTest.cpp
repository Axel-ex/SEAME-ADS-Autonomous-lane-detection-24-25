#include "MotionControlNode.hpp"
#include "gtest/gtest.h"
#include <rclcpp/rclcpp.hpp>

/* MOCK OBJECTS */
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
            return val; // No smoothing
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

/* FIXTURE */
/**
 * @class MotionControlTest
 * @brief Fixture encapuslating init logic and mock objects.
 */
class MotionControlTest : public ::testing::Test
{
    protected:
        std::shared_ptr<MotionControlNode> node_;
        std::shared_ptr<MockPIDController> pid_;
        std::shared_ptr<MockKalmanFilter> kalman_filter_;
        std::shared_ptr<MockLaneBuffer> lane_buffer_;

        void SetUp() override
        {
            pid_ = std::make_shared<MockPIDController>();
            kalman_filter_ = std::make_shared<MockKalmanFilter>();
            lane_buffer_ = std::make_shared<MockLaneBuffer>();

            node_ = std::make_shared<MotionControlNode>(pid_, kalman_filter_,
                                                        lane_buffer_);
        }

        void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(MotionControlTest, ValidLaneInput)
{
    // Create executor
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);

    // Create publisher for lane_position topic
    auto pub = node_->create_publisher<custom_msgs::msg::LanePositions>(
        "lane_position", 10);

    // Create test message
    auto msg = std::make_shared<custom_msgs::msg::LanePositions>();

    geometry_msgs::msg::Point32 p1, p2, p3, p4, p5, p6;
    p1.x = 0;
    p1.y = 0.2;
    p1.z = 0;
    p2.x = 1;
    p2.y = 0.2;
    p2.z = 0;
    p3.x = 2;
    p3.y = 0.2;
    p3.z = 0;

    p4.x = 0;
    p4.y = 0.8;
    p4.z = 0;
    p5.x = 1;
    p5.y = 0.8;
    p5.z = 0;
    p6.x = 2;
    p6.y = 0.8;
    p6.z = 0;

    msg->left_lane = {p1, p2, p3};
    msg->right_lane = {p4, p5, p6};

    // Publish and spin
    pub->publish(*msg);

    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) <
           std::chrono::milliseconds(100))
    {
        exec.spin_some();
    }

    // Expect PID received correct error
    EXPECT_NEAR(pid_->last_error, 0.0, 1e-5); // Midpoint should be centered
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
