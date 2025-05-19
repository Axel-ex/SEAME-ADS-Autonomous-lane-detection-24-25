#include <ImageProcessor.hpp>
#include <opencv2/opencv.hpp>

/**
 * @brief Construct a new ImageProcessor object.
 *
 * Initializes all CUDA filters and detectors using the given input and output
 * sizes.
 *
 * @param input_size Expected input size for the ML model.
 * @param output_size Expected size for post-processed output (if needed).
 */
ImageProcessor::ImageProcessor(const cv::Size& input_size,
                               const cv::Size& output_size)
    : input_size_(input_size), output_size_(output_size)
{
    canny_edge_detector_ =
        cv::cuda::createCannyEdgeDetector(LOW_CANNY, HIGH_CANNY);
    line_detector_ = cv::cuda::createHoughSegmentDetector(
        1.0, CV_PI / 180.0f, MIN_LINE_LENGTH, MAX_LINE_GAP, MAX_DETECTED_LINE);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE * 2 + 1, KERNEL_SIZE * 2 + 1),
        cv::Point(KERNEL_SIZE, KERNEL_SIZE));

    dilation_filter_ =
        cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel);
    erosion_filter_ =
        cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel);
}

/**
 * @brief Resizes and normalizes a BGR image, flattening it into a float vector.
 *
 * The image is resized to the configured input size, and each channel is
 * normalized to [0, 1]. The data is then stored in CHW (all red pixels,
 * followed by all green pixels...)
 *
 * @param img Reference to the input image (cv::Mat).
 * @return std::vector<float> Flattened, normalized image data.
 */
std::vector<float> ImageProcessor::flattenImage(cv::Mat& img) const
{
    cv::resize(img, img, input_size_);
    std::vector<float> flatten_img(3 * input_size_.height * input_size_.width);

    for (int c = 0; c < 3; ++c)
    {
        for (int y = 0; y < img.rows; ++y)
        {
            for (int x = 0; x < img.cols; ++x)
            {
                float val =
                    static_cast<float>(img.at<cv::Vec3b>(y, x)[2 - c]) / 255.0f;
                flatten_img[c * img.rows * img.cols + y * img.cols + x] = val;
            }
        }
    }
    return flatten_img;
}

/**
 * @brief Extracts lines from an edge-detected image using Hough Transform.
 *
 * Performs a probabilistic Hough line transform on the image.
 *
 * @param gpu_img Edge-detected input image.
 * @return std::vector<cv::Vec4i> Vector of detected lines (x1, y1, x2, y2).
 */
std::vector<cv::Vec4i> ImageProcessor::getLines(cv::cuda::GpuMat& gpu_img)
{

    cv::cuda::GpuMat gpu_lines;
    line_detector_->detect(gpu_img, gpu_lines);

    // Convert to vector of Vec4i
    std::vector<cv::Vec4i> lines;
    if (!gpu_lines.empty())
    {
        cv::Mat lines_cpu(1, gpu_lines.cols, CV_32SC4);
        gpu_lines.download(lines_cpu);
        lines.assign(lines_cpu.ptr<cv::Vec4i>(),
                     lines_cpu.ptr<cv::Vec4i>() + lines_cpu.cols);
    }

    return lines;
}

/**
 * @brief Applies a binary threshold on a GPU image.
 *
 * All values above the threshold are set to 255; others are set to 0.
 *
 * @param gpu_img Image on which to apply thresholding (in-place).
 * @param treshold Threshold value.
 */
void ImageProcessor::applyTreshold(cv::cuda::GpuMat& gpu_img, int treshold)
{
    cv::cuda::threshold(gpu_img, gpu_img, treshold, 255, cv::THRESH_BINARY);
}

/**
 * @brief Applies dilation followed by erosion (closing) to reduce noise.
 *
 * This is useful for filling small gaps in detected edges or lines.
 *
 * @param gpu_img Image on which to apply morphological operations (in-place).
 */
void ImageProcessor::applyErosionDilation(cv::cuda::GpuMat& gpu_img)
{
    dilation_filter_->apply(gpu_img, gpu_img);
    erosion_filter_->apply(gpu_img, gpu_img);
}

/**
 * @brief Applies the Canny edge detector to a GPU image.
 *
 * Detects edges in the image using CUDA-accelerated Canny filter.
 *
 * @param gpu_img Image on which to apply edge detection (in-place).
 */
void ImageProcessor::applyCannyEdge(cv::cuda::GpuMat& gpu_img)
{
    canny_edge_detector_->detect(gpu_img, gpu_img);
}
