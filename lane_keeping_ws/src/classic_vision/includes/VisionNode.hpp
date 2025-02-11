#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
/**
 * @class VisionNode
 * @brief Processes the image and estimate distance from the center of the lane
 *
 */
class VisionNode : public rclcpp::Node
{
    public:
        VisionNode();
        ~VisionNode() = default;

        // process:
        // Grayscale
        // blur
        // edges
        // ----> produces edges

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        void processImage(sensor_msgs::msg::Image::SharedPtr img);
        // Subscription to image_topic
        //
};
