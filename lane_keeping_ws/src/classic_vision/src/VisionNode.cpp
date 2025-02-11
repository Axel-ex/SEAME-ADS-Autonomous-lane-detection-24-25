#include "VisionNode.hpp"
#include "cv_bridge/cv_bridge.h"

VisionNode::VisionNode() : Node("vision_node")
{
    auto qos = rclcpp::QoS(60);

    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::processImage(img); });

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void VisionNode::processImage(sensor_msgs::msg::Image::SharedPtr img)
{
    auto converted = cv_bridge::toCvShare(img);

    RCLCPP_INFO(this->get_logger(), "Image received");
}
