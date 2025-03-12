#include "LaneVisualizationNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LaneVisualizationNode>();
    node->initPublishers();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
