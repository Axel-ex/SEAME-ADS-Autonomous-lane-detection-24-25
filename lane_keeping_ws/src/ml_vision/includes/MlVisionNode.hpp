#pragma once

#include <ImageProcessor.hpp>
#include <InferenceEngine.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr int LOG_FREQ = 5000;
const cv::Size INPUT_IMG_SIZE(256, 256);
const cv::Size OUTPUT_IMG_SIZE(256, 256);

/**
 * @class MlVisionNode
 * @brief Performs lane detection using a deep learning model
 *        accelerated with TensorRT and CUDA-based image preprocessing.
 *
 * Responsibilities:
 * - Subscribes to raw camera images.
 * - Runs image preprocessing (resize, flatten).
 * - Executes inference on GPU using TensorRT.
 * - Posprocess image (treshold, morpho operations, Canny edge)
 * - Extracts lane lines from model output via Hough Transform.
 * - Publishes lane lines and debug visualizations.
 */
class MlVisionNode : public rclcpp::Node
{
    public:
        MlVisionNode();
        ~MlVisionNode() = default;

        bool init();

    private:
        // ROS
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Publisher<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_pub_;
        image_transport::Publisher raw_mask_pub_;
        image_transport::Publisher tresholded_mask_pub_;
        image_transport::Publisher edge_mask_pub_;

        // ML and CV
        std::unique_ptr<InferenceEngine> inference_engine_;
        std::unique_ptr<ImageProcessor> image_processor_;

        // Private function member
        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
        void publishLanePositions(std::vector<cv::Vec4i>& lines);

        // Debug
        void publishDebug(cv::cuda::GpuMat& gpu_img,
                          image_transport::Publisher& publisher) const;
};
