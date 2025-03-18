#pragma once

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr int DEBUG_LOG_FREQ_MS = 5000;

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
        image_transport::Publisher edge_img_pub_;
        image_transport::Publisher mask_pub_;

        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);

        void preProcessImage(cv::cuda::GpuMat& gpu_img);
        void applyTreshold(cv::cuda::GpuMat& gpu_img, bool is_white_lane);
        void applyMorphoTransfo(cv::cuda::GpuMat& gpu_img);
        void cropToROI(cv::cuda::GpuMat& gpu_img);

        std::vector<cv::Vec4i> getLines(cv::cuda::GpuMat& gpu_img);
        void publishLines(std::vector<cv::Vec4i>& lines, int img_width);
        void publishMaskImg(cv::cuda::GpuMat& gpu_img);
};
