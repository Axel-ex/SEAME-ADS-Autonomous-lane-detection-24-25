#include "VisionNode.hpp"

VisionNode::VisionNode() : Node("vision_node")
{
    RCLCPP_INFO(this->get_logger(), "Node initialized");
}
