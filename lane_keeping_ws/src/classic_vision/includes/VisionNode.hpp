#pragma once

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
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

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        void processImage(sensor_msgs::msg::Image::SharedPtr img);

        void preProcessImage(cv::cuda::GpuMat& gpu_img);
        void detectLines(cv::cuda::GpuMat& gpu_img,
                         cv_bridge::CvImageConstPtr& original_img);
};
