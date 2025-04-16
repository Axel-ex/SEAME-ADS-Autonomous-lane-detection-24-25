#pragma once

#include <NvInfer.h>
#include <cv_bridge/cv_bridge.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr auto ENGINE_PATH = "model.engine";
constexpr auto INPUT_LAYER_NAME = "input_1";
constexpr auto OUTPUT_LAYER_NAME = "conv2d_14";
constexpr int LOG_FREQ = 5000;

const cv::Size INPUT_IMG_SIZE(256, 256);
const cv::Size OUTPUT_IMG_SIZE(256, 256);

using namespace nvinfer1;

// Custom deleters to be able to work with smart ptr and RAII
template <typename T> struct TrtDeleter
{
        void operator()(T* obj) const
        {
            if (obj)
                obj->destroy();
        }
};

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
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Publisher<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_pub_;

        TrtUniquePtr<IRuntime> runtime_;
        TrtUniquePtr<IExecutionContext> context_;
        TrtUniquePtr<ICudaEngine> engine_;
        size_t input_size_{1};
        size_t output_size_{1};
        CudaUniquePtr<void> d_input_;
        CudaUniquePtr<void> d_output_;

        ICudaEngine* createCudaEngine();
        void allocateDevices();
        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg);
        std::vector<float> flattenImage(cv_bridge::CvImageConstPtr img_ptr);
        std::vector<float> runInference(std::vector<float>& flat_img);
};
