#pragma once

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode();
        ~CameraNode();

    private:
        void captureFrame();

        bool running_;
        std::thread capture_thread_;
        cv::VideoCapture cap_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};
