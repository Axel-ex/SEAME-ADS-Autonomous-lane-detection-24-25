#pragma once

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class CameraNode : public rclcpp::Node
{
    public:
        CameraNode();
        ~CameraNode();

    private:
        void initPublisherAndCapture();
        void captureFrame();

        std::atomic<bool> running_;
        std::thread capture_thread_;
        cv::VideoCapture cap_;
        rclcpp::WallRate capture_rate_{5};
        image_transport::Publisher image_pub_;
};
