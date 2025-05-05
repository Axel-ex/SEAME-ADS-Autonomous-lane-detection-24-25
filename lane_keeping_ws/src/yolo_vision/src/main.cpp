#include "YoloVisionNode.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto vision_node = std::make_shared<YoloVisionNode>();
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(vision_node);
    vision_node->init(); // new method
    exec.spin();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
