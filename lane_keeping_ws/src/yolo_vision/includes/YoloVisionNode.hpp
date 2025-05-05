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

const std::vector<std::string> COCO_CLASSES = {
    "person",        "bicycle",       "car",           "motorbike",
    "aeroplane",     "bus",           "train",         "truck",
    "boat",          "traffic light", "fire hydrant",  "stop sign",
    "parking meter", "bench",         "bird",          "cat",
    "dog",           "horse",         "sheep",         "cow",
    "elephant",      "bear",          "zebra",         "giraffe",
    "backpack",      "umbrella",      "handbag",       "tie",
    "suitcase",      "frisbee",       "skis",          "snowboard",
    "sports ball",   "kite",          "baseball bat",  "baseball glove",
    "skateboard",    "surfboard",     "tennis racket", "bottle",
    "wine glass",    "cup",           "fork",          "knife",
    "spoon",         "bowl",          "banana",        "apple",
    "sandwich",      "orange",        "broccoli",      "carrot",
    "hot dog",       "pizza",         "donut",         "cake",
    "chair",         "sofa",          "pottedplant",   "bed",
    "diningtable",   "toilet",        "tvmonitor",     "laptop",
    "mouse",         "remote",        "keyboard",      "cell phone",
    "microwave",     "oven",          "toaster",       "sink",
    "refrigerator",  "book",          "clock",         "vase",
    "scissors",      "teddy bear",    "hair drier",    "toothbrush"};

struct YoloResult
{
        std::vector<cv::Rect> boxes;
        std::vector<std::string> class_ids;
        std::vector<float> confidences;
};

class YoloVisionNode : public rclcpp::Node
{
    public:
        YoloVisionNode();
        ~YoloVisionNode() = default;

        bool init();

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
        std::string mapIdtoString(int id);
        YoloResult extractResult();
        void publishResult(YoloResult& result);

        void publishLanePositions(std::vector<cv::Vec4i>& lines);
        // Debug
        void publishDebug(cv::cuda::GpuMat& gpu_img,
                          image_transport::Publisher& publisher) const;
};
