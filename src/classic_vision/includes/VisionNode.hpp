#pragma once

#include <rclcpp/rclcpp.hpp>

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
        // Subscription to image_topic
        //
};
