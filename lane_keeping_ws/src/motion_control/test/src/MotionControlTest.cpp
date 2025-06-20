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
            rclcpp::init(0, nullptr);
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

    geometry_msgs::msg::Point32 p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = 100;
    p1.y = 240;
    p2.x = 105;
    p2.y = 180;
    p3.x = 110;
    p3.y = 120;
    p7.x = 115;
    p7.y = 100;

    p4.x = 156;
    p4.y = 240;
    p5.x = 160;
    p5.y = 180;
    p6.x = 165;
    p6.y = 120;
    p8.x = 169;
    p8.y = 100;

    msg->left_lane = {p1, p2, p3, p7};
    msg->right_lane = {p4, p5, p6, p8};
    msg->image_height.data = 256;
    msg->image_width.data = 256;

    // Publish and spin
    pub->publish(*msg);

    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) <
           std::chrono::milliseconds(100))
    {
        exec.spin_some();
    }

    EXPECT_TRUE(pid_->last_error != 0.0);
}

TEST_F(MotionControlTest, RightLaneOnly_UsesBufferForLeft)
{
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);

    auto pub = node_->create_publisher<custom_msgs::msg::LanePositions>(
        "lane_position", 10);

    auto msg = std::make_shared<custom_msgs::msg::LanePositions>();
    msg->image_width.data = 256;
    msg->image_height.data = 256;

    geometry_msgs::msg::Point32 p4, p5, p6, p8;
    p4.x = 156;
    p4.y = 240;
    p5.x = 160;
    p5.y = 180;
    p6.x = 165;
    p6.y = 120;
    p8.x = 169;
    p8.y = 100;

    msg->right_lane = {p4, p5, p6, p8};
    msg->image_height.data = 256;
    msg->image_width.data = 256;
    pub->publish(*msg);
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) <
           std::chrono::milliseconds(100))
        exec.spin_some();

    EXPECT_TRUE(pid_->last_error != 0.0);
}

TEST_F(MotionControlTest, NoLanes_StopsVehicle)
{
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node_);

    auto pub = node_->create_publisher<custom_msgs::msg::LanePositions>(
        "lane_position", 10);

    // Capture cmd_vel output
    geometry_msgs::msg::Twist last_cmd;
    auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        [&](geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd = *msg; });

    auto msg = std::make_shared<custom_msgs::msg::LanePositions>();
    msg->image_width.data = 640;
    msg->image_height.data = 480;

    // No lanes at all
    msg->left_lane.clear();
    msg->right_lane.clear();

    pub->publish(*msg);
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) <
           std::chrono::milliseconds(100))
        exec.spin_some();

    // Expect vehicle stop command
    EXPECT_NEAR(last_cmd.linear.x, 0.0, 1e-4);
    EXPECT_NEAR(last_cmd.angular.z, 0.0, 1e-4);
}
//
// TEST_F(MotionControlTest, HighSteeringIncreasesSpeed)
// {
//     rclcpp::executors::SingleThreadedExecutor exec;
//     exec.add_node(node_);
//
//     auto pub = node_->create_publisher<custom_msgs::msg::LanePositions>(
//         "lane_position", 10);
//
//     geometry_msgs::msg::Twist last_cmd;
//     auto sub = node_->create_subscription<geometry_msgs::msg::Twist>(
//         "cmd_vel", 10,
//         [&](geometry_msgs::msg::Twist::SharedPtr msg) { last_cmd = *msg; });
//
//     // Create test message
//     auto msg = std::make_shared<custom_msgs::msg::LanePositions>();
//     geometry_msgs::msg::Point32 p1, p2, p3, p4, p5, p6, p7, p8;
//
//     p1.x = 10;
//     p1.y = 240;
//     p2.x = 15;
//     p2.y = 180;
//     p3.x = 18;
//     p3.y = 120;
//     p7.x = 20;
//     p7.y = 100;
//
//     p4.x = 60;
//     p4.y = 240;
//     p5.x = 65;
//     p5.y = 180;
//     p6.x = 68;
//     p6.y = 120;
//     p8.x = 70;
//     p8.y = 100;
//     msg->left_lane = {p1, p2, p3, p7};
//     msg->right_lane = {p4, p5, p6, p8};
//     msg->image_height.data = 256;
//     msg->image_width.data = 256;
//
//     pub->publish(*msg);
//     auto start = std::chrono::steady_clock::now();
//     while ((std::chrono::steady_clock::now() - start) <
//            std::chrono::milliseconds(100))
//         exec.spin_some();
//
//     // Expect higher speed due to high steering value (0.6 threshold)
//     EXPECT_GT(last_cmd.linear.x, 0.5);
// }
