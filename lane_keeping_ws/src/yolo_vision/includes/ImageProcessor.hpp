#pragma once
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <rclcpp/rclcpp.hpp>

const cv::Size INPUT_IMG_SIZE(256, 256);
const cv::Size OUTPUT_IMG_SIZE(256, 256);

constexpr int LOW_CANNY = 50;
constexpr int HIGH_CANNY = 80;
constexpr float TRESHOLD = 190;
constexpr int MIN_LINE_LENGTH = 20;
constexpr int MAX_LINE_GAP = 20;
constexpr int MAX_DETECTED_LINE = 300;
constexpr int KERNEL_SIZE = 3;

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
        std::vector<cv::Vec4i> getLines(cv::cuda::GpuMat& gpu_img);

    private:
        cv::Size input_size_;
        cv::Size output_size_;
        cv::Ptr<cv::cuda::CannyEdgeDetector> canny_edge_detector_;
        cv::Ptr<cv::cuda::HoughSegmentDetector> line_detector_;
        cv::Ptr<cv::cuda::Filter> erosion_filter_;
        cv::Ptr<cv::cuda::Filter> dilation_filter_;
};
