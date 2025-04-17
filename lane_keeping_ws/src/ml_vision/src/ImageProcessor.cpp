#include <ImageProcessor.hpp>
#include <opencv2/opencv.hpp>

ImageProcessor::ImageProcessor(const cv::Size& input_size,
                               const cv::Size& output_size)
    : input_size_(input_size), output_size_(output_size)
{
    canny_edge_detector_ =
        cv::cuda::createCannyEdgeDetector(LOW_CANNY, HIGH_CANNY);
    line_detector_ = cv::cuda::createHoughSegmentDetector(
        1.0, 180 / CV_PI, MIN_LINE_LENGTH, MAX_LINE_GAP, MAX_DETECTED_LINE);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(KERNEL_SIZE * 2 + 1, KERNEL_SIZE * 2 + 1),
        cv::Point(KERNEL_SIZE, KERNEL_SIZE));

    dilation_filter_ =
        cv::cuda::createMorphologyFilter(cv::MORPH_DILATE, CV_8UC1, kernel);
    erosion_filter_ =
        cv::cuda::createMorphologyFilter(cv::MORPH_ERODE, CV_8UC1, kernel);
}

/**
 * @brief transform an Image into a flat vector (raw sequence of bytes)
 *
 * @param img_ptr
 * @return
 */
std::vector<float> ImageProcessor::flattenImage(cv::Mat& img) const
{
    cv::resize(img, img, input_size_);
    std::vector<float> flatten_img(input_size_.height * input_size_.width * 3);

    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            cv::Vec3b pixel = img.at<cv::Vec3b>(y, x);
            size_t base = (y * img.cols + x) * 3;
            flatten_img[base + 0] = static_cast<float>(pixel[2] / 255.0);
            flatten_img[base + 1] = static_cast<float>(pixel[1] / 255.0);
            flatten_img[base + 2] = static_cast<float>(pixel[0] / 255.0);
        }
    }
    return flatten_img;
}

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

void ImageProcessor::applyTreshold(cv::cuda::GpuMat& gpu_img, int treshold)
{
    cv::cuda::threshold(gpu_img, gpu_img, treshold, 255, cv::THRESH_BINARY);
}

void ImageProcessor::applyErosionDilation(cv::cuda::GpuMat& gpu_img)
{
    dilation_filter_->apply(gpu_img, gpu_img);
    erosion_filter_->apply(gpu_img, gpu_img);
}

void ImageProcessor::applyCannyEdge(cv::cuda::GpuMat& gpu_img)
{
    canny_edge_detector_->detect(gpu_img, gpu_img);
}
