#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <image_transport/image_transport.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <lane_msgs/msg/polyfit_coefs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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

        void initPublishers();

    private:
        std::vector<double> polyfit_coefs;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Subscription<lane_msgs::msg::PolyfitCoefs>::SharedPtr
            polyfit_coefs_sub_;
        image_transport::Publisher img_pub_;

        void processImage(const sensor_msgs::msg::Image::SharedPtr msg);
        void storeCoefs(const lane_msgs::msg::PolyfitCoefs::SharedPtr msg);
};
