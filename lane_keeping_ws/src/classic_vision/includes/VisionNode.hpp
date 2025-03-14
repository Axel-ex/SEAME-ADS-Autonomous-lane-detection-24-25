#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr int DEBUG_LOG_FREQ_MS = 5000;
constexpr float IMG_WIDTH = 640;
constexpr float IMG_HEIGHT = 480;

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

        void initPublisher();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Publisher<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_pub_;
        image_transport::Publisher processed_img_pub_;
        image_transport::Publisher edge_img_pub_;
        image_transport::Publisher orange_mask_pub_;

        void processImage(sensor_msgs::msg::Image::SharedPtr img_msg);
        void applyTreshold(cv::cuda::GpuMat& gpu_img);
        void preProcessImage(cv::cuda::GpuMat& gpu_img);
        void cropToROI(cv::cuda::GpuMat& gpu_img);

        std::vector<cv::Vec4i> getLines(cv::cuda::GpuMat& gpu_img);
        void drawLines(cv::Mat& original_img, std::vector<cv::Vec4i>& lines);
        void publishLines(std::vector<cv::Vec4i>& lines);
};
