#pragma once

#include <NvInfer.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

// TENSORRT
constexpr auto ENGINE_PATH =
    "/home/axel/SEAME-ADS-Autonomous-lane-detection-24-25/model.engine";
constexpr auto INPUT_LAYER_NAME = "input_1";
constexpr auto OUTPUT_LAYER_NAME = "conv2d_14";
const cv::Size INPUT_IMG_SIZE(256, 256);
const cv::Size OUTPUT_IMG_SIZE(256, 256);

// OPENCV
constexpr int LOW_CANNY = 50;
constexpr int HIGH_CANNY = 80;
constexpr float TRESHOLD = 120;

constexpr int MIN_LINE_LENGTH = 20;
constexpr int MAX_LINE_GAP = 20;
constexpr int MAX_DETECTED_LINE = 300;

constexpr int KERNEL_SIZE = 3;
constexpr int LOG_FREQ = 5000;

using namespace nvinfer1;

// Custom deleters to be able to work with smart ptr and RAII
// Calling destroy raise warnings due to depreciation but with the tensoRT
// version we are working with, it still safer not to call delete on those
// objects

/**
 * @brief custom deleter for TRT objects
 *
 * @tparam T
 * @param obj
 */
template <typename T> struct TrtDeleter
{
        void operator()(T* obj) const
        {
            if (obj)
                obj->destroy();
        }
};

/**
 * @brief custom deleter for Cuda allocations
 *
 * @tparam T
 * @param ptr
 */
template <typename T> struct cudaDeleter
{
        void operator()(T* ptr) const
        {
            if (ptr)
            {
                cudaError_t error = cudaFree(ptr);
                if (error != cudaSuccess)
                    std::cerr << cudaGetErrorString(error) << "\n";
            }
        }
};

// Alias to make it easier to use
template <typename T> using TrtUniquePtr = std::unique_ptr<T, TrtDeleter<T>>;
template <typename T> using CudaUniquePtr = std::unique_ptr<T, cudaDeleter<T>>;

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
        image_transport::Publisher edge_img_pub_;
        image_transport::Publisher raw_mask_pub_;
        image_transport::Publisher processed_mask_pub_;

        // tensoRT
        TrtUniquePtr<IRuntime> runtime_;
        TrtUniquePtr<IExecutionContext> context_;
        TrtUniquePtr<ICudaEngine> engine_;
        size_t input_size_{1};
        size_t output_size_{1};
        CudaUniquePtr<void> d_input_;
        CudaUniquePtr<void> d_output_;

        // OpenCV (lane extraction from inference result)
        cv::Ptr<cv::cuda::CannyEdgeDetector> canny_edge_detector_;
        cv::Ptr<cv::cuda::HoughSegmentDetector> line_detector_;
        cv::Ptr<cv::cuda::Filter> erosion_filter_;
        cv::Ptr<cv::cuda::Filter> dilation_filter_;

        // Private function member
        ICudaEngine* createCudaEngine();
        void allocateDevices();
        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
        std::vector<float>
        flattenImage(cv_bridge::CvImageConstPtr img_ptr) const;
        std::vector<float>
        runInference(const std::vector<float>& flat_img) const;
        void postProcessing(cv::cuda::GpuMat& gpu_img);
        std::vector<cv::Vec4i> getLines(cv::cuda::GpuMat& gpu_img);
        void publishLanePositions(std::vector<cv::Vec4i>& lines);

        // Debug
        void publishDebug(cv::cuda::GpuMat& gpu_img,
                          image_transport::Publisher& publisher) const;
};
