#pragma once

#include <NvInfer.h>
#include <array>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace nvinfer1;

constexpr auto ENGINE_PATH = "/home/axel/models/engines/yolop-640-640.engine";
constexpr auto INPUT_LAYER_NAME = "images";
constexpr auto OUTPUT_OBJECT_DETECTION = "det_out";
constexpr auto OUTPUT_ROAD_SEGMENTATION = "drive_area_seg";
constexpr auto OUTPUT_LANE_SEGMENTATION = "lane_line_seg";

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

template <typename T> using TrtUniquePtr = std::unique_ptr<T, TrtDeleter<T>>;
template <typename T> using CudaUniquePtr = std::unique_ptr<T, cudaDeleter<T>>;

/**
 * @class InferenceEngine
 * @brief Encapsulates TensorRT engine creation, memory management, and
 * inference execution.
 *
 * This class manages the full lifecycle of the TensorRT engine, including
 * deserialization, context creation, device memory allocation, and inference
 * execution. The inference result is kept on the GPU to allow efficient
 * post-processing.
 */
class InferenceEngine
{
    public:
        InferenceEngine(std::shared_ptr<rclcpp::Node> node_ptr);
        ~InferenceEngine();

        bool init();
        bool runInference(const std::vector<float>& flat_img) const;
        void checkEngineSpecs();

        std::array<float*, 3> getOutputDevicePtrs() const;
        std::array<size_t, 3> getOuputSizes() const;
        size_t getInputSize() const;

    private:
        TrtUniquePtr<IRuntime> runtime_;
        TrtUniquePtr<IExecutionContext> context_;
        TrtUniquePtr<ICudaEngine> engine_;
        std::shared_ptr<rclcpp::Node> node_ptr_;

        // YOLOP: multi output. 3 different output with 3 differents sizes to
        // keep track of: objects detection, road surface segmentation and lane
        // segmentation
        size_t input_size_{1};
        size_t output_det_size_{1};
        size_t output_road_seg_size_{1};
        size_t output_lane_seg_size_{1};
        CudaUniquePtr<void> d_input_;
        CudaUniquePtr<void> d_output_det_;
        CudaUniquePtr<void> d_output_road_seg_;
        CudaUniquePtr<void> d_output_lane_seg_;

        ICudaEngine* createCudaEngine();
        void allocateDevices();
};
