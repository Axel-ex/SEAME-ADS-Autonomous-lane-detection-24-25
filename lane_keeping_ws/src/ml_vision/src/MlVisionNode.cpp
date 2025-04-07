#include <MlVisionNode.hpp>

MlVisionNode::MlVisionNode() : rclcpp::Node("ml_vision_node")
{
    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "raw_img", 1,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { this->rawImageCallback(img); });

    lane_pos_pub_ = this->create_publisher<lane_msgs::msg::LanePositions>(
        "lane_positions", 10);
}

void MlVisionNode::rawImageCallback(sensor_msgs::msg::Image::SharedPtr img) {}
