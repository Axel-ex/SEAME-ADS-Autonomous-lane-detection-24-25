#include "MotionControlNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotionControlNode>();
    node->initPIDController();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
