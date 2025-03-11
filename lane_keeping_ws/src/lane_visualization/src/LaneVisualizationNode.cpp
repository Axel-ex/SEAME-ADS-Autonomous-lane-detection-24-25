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
    ColorRGBA lane_color;
    lane_color.a = 1.0;
    lane_color.g = 0.1;
    lane_color.b = 0.0;
    lane_color.r = 0.0;

    publishLaneMarks(msg->right_lane, lane_color);
    lane_color.b = 0.1;
    publishLaneMarks(msg->left_lane, lane_color);
}

void LaneVisualizationNode::publishLaneMarks(const std::vector<Point32>& points,
                                             ColorRGBA& color)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker center_mark;
    center_mark.header.frame_id = "map";
    center_mark.header.stamp = this->now();
    center_mark.id = 1;
    center_mark.type = visualization_msgs::msg::Marker::POINTS;
    center_mark.action = visualization_msgs::msg::Marker::ADD;
    center_mark.scale.x = 0.1; // Sphere size
    center_mark.scale.y = 0.1;
    center_mark.scale.z = 0.1;
    center_mark.color.a = 1.0; // transparency
    center_mark.color.r = 0.0;
    center_mark.color.g = 0.1;
    center_mark.color.b = 0.0;

    for (const auto& point : points)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;

        center_mark.points.push_back(p);
    }
    marker_array.markers.push_back(center_mark);
    marker_pub_->publish(marker_array);
}
