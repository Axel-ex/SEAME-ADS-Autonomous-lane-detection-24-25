#pragma once

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

/**
 * @class CameraNode
 * @brief Captures and publish camera frames using OpenCV
 * with GStreamer.
 *
 * This node initializes a camera pipeline (using nvidia camera stack and
 * gstreamer), captures frames in a dedicated thread, and publishes them as ROS2
 * sensor_msgs/Image messages on the "image_raw" topic.
 */
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
        image_transport::Publisher image_pub_;
};
