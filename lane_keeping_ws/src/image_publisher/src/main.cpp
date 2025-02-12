#include "ImagePublisherNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto vision_node = std::make_shared<ImagePublisherNode>();
    rclcpp::spin(vision_node);

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
