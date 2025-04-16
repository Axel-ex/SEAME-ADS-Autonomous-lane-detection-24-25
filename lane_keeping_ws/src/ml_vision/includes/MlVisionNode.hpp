#pragma once

#include <NvInfer.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

constexpr auto engine_path = "model.engine";

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

        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img);
        TrtUniquePtr<IRuntime> runtime_;
        TrtUniquePtr<IExecutionContext> context_;
        TrtUniquePtr<ICudaEngine> engine_;
        CudaUniquePtr<void> d_input_;
        CudaUniquePtr<void> d_output_;

        ICudaEngine* createCudaEngine();
};
