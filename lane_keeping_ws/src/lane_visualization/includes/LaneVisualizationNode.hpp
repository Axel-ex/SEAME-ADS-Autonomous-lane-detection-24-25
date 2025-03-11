#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using Point32 = geometry_msgs::msg::Point32;
using ColorRGBA = std_msgs::msg::ColorRGBA;

class LaneVisualizationNode : public rclcpp::Node
{
    public:
        LaneVisualizationNode();
        ~LaneVisualizationNode() = default;

    private:
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
            marker_pub_;

        void
        processLanePosition(const lane_msgs::msg::LanePositions::SharedPtr msg);
        void publishLaneMarks(const std::vector<Point32>& points,
                              ColorRGBA& color);
};
