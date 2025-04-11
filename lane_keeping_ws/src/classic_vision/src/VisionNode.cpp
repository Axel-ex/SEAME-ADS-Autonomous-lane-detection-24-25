#include "VisionNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <lane_msgs/msg/lane_positions.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

/**
 *  * @brief Initializes the VisionNode with ROS2 subscriptions, publishers, and
 * parameters.
 *
 * - Subscribes to `image_raw` (best-effort QoS).
 * - Publishes `lane_position` (LanePositions messages).
 * - Declares parameters for lane detection tuning:
 *   - `max_detected_lines`, `low/high_canny_treshold`, `rho`
 *   - `is_white_lane`, `treshold_sensitivity`
 */
VisionNode::VisionNode() : Node("vision_node")
{
    auto qos = rclcpp::QoS(QOS).best_effort();
    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::rawImageCallback(img); });
    lane_pos_pub_ = this->create_publisher<lane_msgs::msg::LanePositions>(
        "lane_position", 10);

    this->declare_parameter("max_detected_lines", 200);
    this->declare_parameter("low_canny_treshold", 50);
    this->declare_parameter("high_canny_treshold", 80);
    this->declare_parameter("rho", 1.0);
    this->declare_parameter("is_white_lane", true);
    this->declare_parameter("treshold_sensitivity", 80);
    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void VisionNode::initPublisher()
{
    auto it = image_transport::ImageTransport(shared_from_this());
    edge_img_pub_ = it.advertise("edges_img", 1);
    mask_pub_ = it.advertise("mask_img", 1);
}

/**
 * @brief Callback for processing incoming camera images.
 *
 * - Converts ROS2 Image message to OpenCV format (GPU-accelerated).
 * - Applies preprocessing, line detection, and lane position estimation.
 * - Handles empty images and OpenCV exceptions gracefully.
 *
 * @param img_msg Shared pointer to the incoming sensor_msgs/Image.
 */
void VisionNode::rawImageCallback(sensor_msgs::msg::Image::SharedPtr img_msg)
{
    try
    {
        auto converted = cv_bridge::toCvShare(img_msg, img_msg->encoding);
        if (converted->image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received an empty image!");
            return;
        }

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(),
                              DEBUG_LOG_FREQ_MS, "Input image size: %dx%d",
                              converted->image.cols, converted->image.rows);

        // Upload the image to GPU
        cuda::GpuMat gpu_img;
        gpu_img.upload(converted->image);

        preProcessImage(gpu_img);
        auto lines = getLines(gpu_img);
        publishLanePositions(lines, gpu_img.cols, gpu_img.rows);
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while processing the image: %s",
                     e.what());
    }
}

/**
 * @brief Applies GPU-accelerated preprocessing pipeline to an image.
 *
 * Steps:
 * 1. Thresholding (adaptive based on `is_white_lane` parameter).
 * 2. Morphological transformations (dilation + erosion).
 * 3. ROI cropping (trapezoidal mask).
 * 4. Canny edge detection.
 *
 * @param gpu_img Input/Output image in GPU memory (cuda::GpuMat).
 */
void VisionNode::preProcessImage(cuda::GpuMat& gpu_img)
{
    auto is_white_lane = get_parameter("is_white_lane").as_bool();

    applyTreshold(gpu_img, is_white_lane);
    applyMorphoTransfo(gpu_img);
    cropToROI(gpu_img);
    publishMaskImg(gpu_img);
    applyCannyEdge(gpu_img);
    // publishEdgeImage(gpu_img);
}

/**
 * @brief Detects lines using GPU-accelerated Hough transform.
 *
 * - Configures Hough detector with `max_detected_lines` and `rho` parameters.
 * - Converts GPU results to CPU vector<Vec4i>.
 *
 * @param gpu_img Preprocessed edge image (GPU memory).
 * @return Vector of detected lines (each as Vec4i: x1, y1, x2, y2).
 */
std::vector<Vec4i> VisionNode::getLines(cuda::GpuMat& gpu_img)
{
    auto max_detected_lines =
        this->get_parameter("max_detected_lines").as_int();
    auto rho = this->get_parameter("rho").as_double();

    cuda::GpuMat gpu_lines;
    auto line_detector = cuda::createHoughSegmentDetector(
        rho, CV_PI / 180.0f, 20, 20, max_detected_lines);

    line_detector->detect(gpu_img, gpu_lines);

    // Convert to vector of Vec4i
    std::vector<Vec4i> lines;
    if (!gpu_lines.empty())
    {
        Mat lines_cpu(1, gpu_lines.cols, CV_32SC4);
        gpu_lines.download(lines_cpu);
        lines.assign(lines_cpu.ptr<Vec4i>(),
                     lines_cpu.ptr<Vec4i>() + lines_cpu.cols);
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(),
                          DEBUG_LOG_FREQ_MS, "Detected %zu valid lines",
                          lines.size());
    return lines;
}

/**
 * @brief Classifies lines as left/right lanes and publishes results.
 *
 * - Filters horizontal lines (slope < 0.3).
 * - Splits lines by slope and image position.
 * - Constructs LanePositions message with:
 *   - Left/right lane points (geometry_msgs/Point32).
 *   - Original image dimensions.
 *
 * @param lines Detected lines from Hough transform.
 * @param img_width, img_height Source image dimensions.
 */
void VisionNode::publishLanePositions(std::vector<cv::Vec4i>& lines,
                                      int img_width, int img_height)
{
    lane_msgs::msg::LanePositions msg;
    msg.header.stamp = this->now();

    std::vector<cv::Vec4i> left_lines, right_lines;

    for (const auto& line : lines)
    {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double slope = static_cast<double>(y2 - y1) / (x2 - x1);

        // remove horizontal lines
        if (std::abs(slope) < 0.3)
            continue;

        // Classify lines based on slope and position
        if (slope < 0 && x1 < (img_width / 2))
            left_lines.push_back(line);
        else if (slope > 0 && x1 > (img_width / 2))
            right_lines.push_back(line);
    }

    // Add left lane lines to the message
    for (auto& line : left_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.y = line[3];
        msg.left_lane.insert(msg.left_lane.end(), {p1, p2});
    }

    // Add right lane lines to the message
    for (auto& line : right_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.y = line[3];
        msg.right_lane.insert(msg.right_lane.end(), {p1, p2});
    }

    msg.image_width.data = img_width;
    msg.image_height.data = img_height;

    lane_pos_pub_->publish(msg);
}

/**
 * @brief Applies white/yellow lane thresholding based on `is_white_lane`
 * parameter.
 *
 * @param gpu_img
 * @param is_white_lane
 */
void VisionNode::applyTreshold(cuda::GpuMat& gpu_img, bool is_white_lane)
{
    auto sensitivity = get_parameter("treshold_sensitivity").as_int();

    if (is_white_lane)
    {
        cuda::cvtColor(gpu_img, gpu_img, COLOR_BGR2GRAY);
        int threshold_value = 255 - sensitivity;
        cv::cuda::compare(gpu_img, threshold_value, gpu_img, cv::CMP_GT);
    }
    else
    {
        cuda::cvtColor(gpu_img, gpu_img, COLOR_BGR2HSV);
        Scalar lower_bound(10, 100, 100);
        Scalar upper_bound(30, 255, 255);
        cuda::inRange(gpu_img, lower_bound, upper_bound, gpu_img);
    }
}

/**
 * @brief Applies dilation + erosion (closing) to reduce noise in binary image.
 *
 * @param gpu_img
 */
void VisionNode::applyMorphoTransfo(cuda::GpuMat& gpu_img)
{
    // Morphological transformation
    auto morpho_size = 3;
    auto morph_element = cv::MORPH_ELLIPSE;

    // create the structuring element (kernel)
    Mat elem = getStructuringElement(
        morph_element, Size(2 * morpho_size + 1, 2 * morpho_size + 1),
        Point(morpho_size, morpho_size));
    cuda::GpuMat gpu_elem(elem);

    // Create the filter to erode / dilate for better results
    auto dilate_filter =
        cuda::createMorphologyFilter(cv::MORPH_DILATE, gpu_img.type(), elem);
    auto erode_filter =
        cuda::createMorphologyFilter(cv::MORPH_ERODE, gpu_img.type(), elem);

    // Apply dilation followed by erosion (closing)
    dilate_filter->apply(gpu_img, gpu_img);
    erode_filter->apply(gpu_img, gpu_img);
}

/**
 * @brief Crops image to ROI (hardcoded dimensions).
 *
 * @param gpu_img
 */
void VisionNode::cropToROI(cuda::GpuMat& gpu_img)
{
    int width = gpu_img.cols;
    int height = gpu_img.rows;

    Mat roi_mask = Mat::zeros(height, width, CV_8UC1);

    // Define the region of interest as a polygon (example: trapezoid shape)
    std::vector<cv::Point> roi_points = {
        cv::Point(0, height),         // Bottom-left
        cv::Point(width, height),     // Bottom-right
        cv::Point(width, height / 3), // Top-right
        cv::Point(0, height / 3)      // Top-left
    };
    // Fill the ROI mask with white inside the polygon
    cv::fillPoly(roi_mask, std::vector<std::vector<cv::Point>>{roi_points},
                 cv::Scalar(255));

    // Upload ROI mask to GPU
    cuda::GpuMat gpu_roi_mask;
    gpu_roi_mask.upload(roi_mask);

    // Apply ROI mask using bitwise_and
    cuda::bitwise_and(gpu_img, gpu_roi_mask, gpu_img);
}

/**
 * @brief Runs GPU-accelerated Canny edge detection with parameterized
 * thresholds.
 *
 * @param gpu_img
 */
void VisionNode::applyCannyEdge(cuda::GpuMat& gpu_img)
{
    // Apply Gaussian blur
    Ptr<cuda::Filter> gaussian_filter =
        cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(3, 3), 1.0);
    gaussian_filter->apply(gpu_img, gpu_img);

    // Apply Canny edge detection
    auto lower_canny_treshold =
        this->get_parameter("low_canny_treshold").as_int();
    auto upper_canny_treshold =
        this->get_parameter("high_canny_treshold").as_int();

    Ptr<cuda::CannyEdgeDetector> canny = cuda::createCannyEdgeDetector(
        lower_canny_treshold, upper_canny_treshold);
    canny->detect(gpu_img, gpu_img);
}

void VisionNode::publishMaskImg(cuda::GpuMat& gpu_img)
{
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    Mat mask;

    gpu_img.download(mask);
    hdr.stamp = now();
    msg = cv_bridge::CvImage(hdr, "mono8", mask).toImageMsg();
    mask_pub_.publish(msg);
}

void VisionNode::publishEdgeImage(cuda::GpuMat& gpu_img)
{
    Mat cpu_edges;
    gpu_img.download(cpu_edges);
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(hdr, "mono8", cpu_edges).toImageMsg();
    edge_img_pub_.publish(msg);
}
