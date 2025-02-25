#include "LaneVisualizationNode.hpp"

LaneVisualizationNode::LaneVisualizationNode()
    : rclcpp::Node("lane visualization node")
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_positions", 10,
        [this](const lane_msgs::msg::LanePositions::SharedPtr msg)
        { this->processLanePosition(msg); });
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "lane_marks", 10);
}

void LaneVisualizationNode::processLanePosition(
    const lane_msgs::msg::LanePositions::SharedPtr msg)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker left_center_mark;
    left_center_mark.header.frame_id = "map";
    left_center_mark.header.stamp = this->now();
    left_center_mark.id = 1;
    left_center_mark.type = visualization_msgs::msg::Marker::POINTS;
    left_center_mark.action = visualization_msgs::msg::Marker::ADD;
    left_center_mark.scale.x = 0.1; // Sphere size
    left_center_mark.scale.y = 0.1;
    left_center_mark.scale.z = 0.1;
    left_center_mark.color.a = 1.0; // transparency
    left_center_mark.color.r = static_cast<float>(rand()) / RAND_MAX;
    left_center_mark.color.g = static_cast<float>(rand()) / RAND_MAX;
    left_center_mark.color.b = static_cast<float>(rand()) / RAND_MAX;

    for (const auto& point : msg->left_lane)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;

        left_center_mark.points.push_back(p);
    }
    marker_array.markers.push_back(left_center_mark);
    marker_pub_->publish(marker_array);
}
