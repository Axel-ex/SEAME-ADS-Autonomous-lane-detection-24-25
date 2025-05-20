#pragma once
#include "PerspectiveMapper.hpp"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @class ImageProcessor
 * @brief Handles image preprocessing and feature extraction on the GPU using
 * OpenCV CUDA.
 *
 * This class provides functionality for preparing images for machine learning
 * inference, including resizing, flattening, edge detection, thresholding,
 * morphological operations, and line detection via Hough Transform.
 */
class ImageProcessor
{
    public:
        ImageProcessor(const cv::Size& input_size, const cv::Size& output_size);
        ~ImageProcessor() = default;

        std::vector<float> flattenImage(cv::Mat& img) const;
        void applyTreshold(cv::cuda::GpuMat& gpu_img, int treshold);
        void applyErosionDilation(cv::cuda::GpuMat& gpu_img);
        void applyCannyEdge(cv::cuda::GpuMat& gpu_img);
        cv::cuda::GpuMat applyPerspectiveTransform(cv::cuda::GpuMat& gpu_img);
        std::vector<cv::Vec4i> getLines(cv::cuda::GpuMat& gpu_img);

    private:
        cv::Size input_size_;
        cv::Size output_size_;

        // POSTPROCESSING
        PerspectiveMapper ipm_;
        cv::Ptr<cv::cuda::CannyEdgeDetector> canny_edge_detector_;
        cv::Ptr<cv::cuda::HoughSegmentDetector> line_detector_;
        cv::Ptr<cv::cuda::Filter> erosion_filter_;
        cv::Ptr<cv::cuda::Filter> dilation_filter_;
};
