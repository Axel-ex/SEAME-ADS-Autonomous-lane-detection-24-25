#include "VisionNode.hpp"
#include <functional>

VisionNode::VisionNode() : Node("vision_node")
{
    auto qos = rclcpp::QoS(60);
    //
    // this->create_subscription<sensor_msgs::msg::Image>(
    //     "image_raw", qos,
    //     std::bind(&VisionNode::processImage, this, std::placeholders::_1));

    this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::processImage(img); });

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void VisionNode::processImage(sensor_msgs::msg::Image::SharedPtr img)
{
    (void)img;
    RCLCPP_INFO(this->get_logger(), "Image received");
}
