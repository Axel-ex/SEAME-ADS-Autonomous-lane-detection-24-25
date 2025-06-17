#include "PIDController.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

class PIDControllerTest : public testing::Test
{
    protected:
        rclcpp::Node::SharedPtr test_node_;
        PIDController pid;

        void SetUp() override
        {
            rclcpp::init(0, nullptr);
            test_node_ = std::make_shared<rclcpp::Node>("pid_test_node");

            test_node_->declare_parameter("kp", 1.0);
            test_node_->declare_parameter("ki", 0.1);
            test_node_->declare_parameter("kd", 0.01);

            pid.initializePID(test_node_);
        }

        void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(PIDControllerTest, CalculatesBasicPID)
{
    double result1 = pid.calculate(1.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double result2 = pid.calculate(0.5);

    EXPECT_GT(result1, 0.0); // First result should be non-zero
    EXPECT_GT(result2, 0.0); // Should respond to smaller error
}

/**
 * @brief test for integral term overflow
 */
TEST_F(PIDControllerTest, IntegralClamping)
{
    for (int i = 0; i < 50; ++i)
    {
        pid.calculate(10.0); // drive integral error up
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    double result = pid.calculate(10.0);

    EXPECT_LT(result, 1e4); // sanity check
}

/**
 * @brief Test for derivative term response
 */
TEST_F(PIDControllerTest, DerivativeResponse)
{
    pid.calculate(0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    double output = pid.calculate(10.0); // large delta = high derivative

    EXPECT_GT(output, 1.0); // Should be bigger than pure P gain (1.0 * 10 = 10)
}
