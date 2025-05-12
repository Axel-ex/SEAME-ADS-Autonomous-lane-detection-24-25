#pragma once

#include <ImageProcessor.hpp>
#include <InferenceEngine.hpp>
#include <custom_msgs/msg/lane_positions.hpp>
#include <custom_msgs/msg/yolo_result.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr int LOG_FREQ = 5000;
const std::vector<std::string> YOLOP_CLASSES = {"car"};

struct YoloResult
{
        std::vector<cv::Rect> boxes;
        std::vector<std::string> class_ids;
        std::vector<float> confidences;
};

class YolopVisionNode : public rclcpp::Node
{
    public:
        YolopVisionNode();
        ~YolopVisionNode() = default;

        bool init(std::unique_ptr<InferenceEngine> mock_engine = nullptr,
                  std::unique_ptr<ImageProcessor> mock_image_proc = nullptr);

    private:
        // ROS
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Publisher<custom_msgs::msg::LanePositions>::SharedPtr
            lane_pos_pub_; // TODO: custom message for object detection
        rclcpp::Publisher<custom_msgs::msg::YoloResult>::SharedPtr
            yolo_result_pub_;
        image_transport::Publisher processed_img_pub_;

        // ML and CV
        std::unique_ptr<InferenceEngine> inference_engine_;
        std::unique_ptr<ImageProcessor> image_processor_;

        // Private function member
        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
        YoloResult extractObjectDetectionResult();
        cv::Mat extractLaneMask();
        void publishResult(YoloResult& result);
        void publishDebug(YoloResult& result, cv::Mat& og_img,
                          cv::Mat& lane_mask, std::string& encoding);

        // helpers
        std::string mapIdtoString(int id);
        void publishDebug(cv::cuda::GpuMat& gpu_img,
                          image_transport::Publisher& publisher) const;

        // needed?
        void publishLanePositions(std::vector<cv::Vec4i>& lines);
};
