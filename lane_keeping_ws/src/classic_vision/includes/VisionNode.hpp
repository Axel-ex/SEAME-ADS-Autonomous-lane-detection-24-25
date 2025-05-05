#pragma once

#include <custom_msgs/msg/lane_positions.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr int DEBUG_LOG_FREQ_MS = 5000;
constexpr int QOS = 1;

/**
 * @class VisionNode
 * @brief Processes camera images to detect lane positions using
 * GPU-accelerated OpenCV.
 *
 * Subscribes to raw camera images, applies thresholding, morphological
 * operations, and Hough line detection to identify lane markings. Publishes
 * detected lane positions as `lane_msgs/msg/LanePositions` and optional debug
 * images (edges/mask).
 */
class VisionNode : public rclcpp::Node
{
    public:
        VisionNode();
        ~VisionNode() = default;

        void initPublisher();

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Publisher<custom_msgs::msg::LanePositions>::SharedPtr
            lane_pos_pub_;
        image_transport::Publisher edge_img_pub_;
        image_transport::Publisher mask_pub_;

        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);

        void preProcessImage(cv::cuda::GpuMat& gpu_img);
        void applyTreshold(cv::cuda::GpuMat& gpu_img, bool is_white_lane);
        void applyMorphoTransfo(cv::cuda::GpuMat& gpu_img);
        void cropToROI(cv::cuda::GpuMat& gpu_img);
        void applyCannyEdge(cv::cuda::GpuMat& gpu_img);
        std::vector<cv::Vec4i> getLines(cv::cuda::GpuMat& gpu_img);

        void publishLanePositions(std::vector<cv::Vec4i>& lines, int img_width,
                                  int img_height);
        void publishMaskImg(cv::cuda::GpuMat& gpu_img);
        void publishEdgeImage(cv::cuda::GpuMat& gpu_img);
};
