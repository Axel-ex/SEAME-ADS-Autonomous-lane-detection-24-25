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

/**
 * @class LaneVisualizationNode
 * @brief Display the geometries published by the vision and motion node (lane
 * positions, polyfits...)
 *
 * Subscribe to lane position and polyfit coefficient. When new values arrive,
 * they are stored in member variables. Subscribe to raw_image, draws the stored
 * geometries on the orignal picture and publish the result in processed_img.
 *
 */
class LaneVisualizationNode : public rclcpp::Node
{
    public:
        LaneVisualizationNode();
        ~LaneVisualizationNode() = default;

        void initPublishers();

    private:
        std::vector<double> left_coefs_;
        std::vector<double> right_coefs_;
        std::vector<Point32> left_lane_pos_;
        std::vector<Point32> right_lane_pos_;
        Point32 lane_center_;

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;
        rclcpp::Subscription<lane_msgs::msg::PolyfitCoefs>::SharedPtr
            polyfit_coefs_sub_;
        image_transport::Publisher processed_img_pub_;

        void rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
        void
        storeLanePosition(const lane_msgs::msg::LanePositions::SharedPtr msg);
        void storeCoefs(const lane_msgs::msg::PolyfitCoefs::SharedPtr msg);
};
