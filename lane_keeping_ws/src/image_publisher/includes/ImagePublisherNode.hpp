#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisherNode : public rclcpp::Node
{
    public:
        ImagePublisherNode();
        ~ImagePublisherNode() = default;

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        void publishImage();
};
