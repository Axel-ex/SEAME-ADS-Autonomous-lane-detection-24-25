#include "VisionNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <lane_msgs/msg/lane_positions.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

VisionNode::VisionNode() : Node("vision_node")
{
    auto qos = rclcpp::QoS(60); // TODO: is QOS adapted?
    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::processImage(img); });
    lane_pos_pub_ = this->create_publisher<lane_msgs::msg::LanePositions>(
        "lane_position", 10);

    this->declare_parameter("min_line_length", 50);
    this->declare_parameter("max_line_gap", 20);
    this->declare_parameter("max_detected_lines", 200);
    this->declare_parameter("low_canny_treshold", 20);
    this->declare_parameter("high_canny_treshold", 80);
    this->declare_parameter("rho", 1.0);
    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void VisionNode::initPublisher()
{

    auto it = image_transport::ImageTransport(shared_from_this());
    processed_img_pub_ = it.advertise("processed_img", 1);
    edge_img_pub_ = it.advertise("edges_img", 1);
    orange_mask_pub_ = it.advertise("mask_img", 1);
}

void VisionNode::processImage(sensor_msgs::msg::Image::SharedPtr img_msg)
{
    try
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(),
                              DEBUG_LOG_FREQ_MS, "Received image: %s",
                              img_msg->encoding.c_str());

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
        publishLines(lines);
        Mat original_img = converted->image.clone();
        drawLines(original_img, lines);
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while processing the image: %s",
                     e.what());
    }
}

void VisionNode::applyTreshold(cuda::GpuMat& gpu_img)
{
    cuda::cvtColor(gpu_img, gpu_img, COLOR_BGR2HSV);
    // H: hue, 0-179 color type
    // S: Saturation, 0-255 how vivid/pure
    // V: value, 0-255 how brght

    // define the HSV range
    Scalar lower_orange(0, 80, 80);
    Scalar upper_orange(15, 255, 255);

    // Apply mask. instead of creating a mask and masking the original img, we
    // use the mask as the picture for further processing
    cuda::inRange(gpu_img, lower_orange, upper_orange, gpu_img);

    // Morphological transformation
    auto morpho_size = 3;
    auto morph_element = cv::MORPH_ELLIPSE;

    // create the structuring element (kernel)
    Mat elem = getStructuringElement(
        morph_element, Size(2 * morpho_size + 1, 2 * morpho_size + 1),
        Point(morpho_size, morpho_size));

    cuda::GpuMat gpu_elem(elem);

    // Create the cuda filters
    auto dilate_filter =
        cuda::createMorphologyFilter(cv::MORPH_DILATE, gpu_img.type(), elem);
    auto erode_filter =
        cuda::createMorphologyFilter(cv::MORPH_ERODE, gpu_img.type(), elem);

    // Apply dilation followed by erosion (closing)
    dilate_filter->apply(gpu_img, gpu_img);
    erode_filter->apply(gpu_img, gpu_img);

    cropToROI(gpu_img);
    // Publish result
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    Mat mask;
    gpu_img.download(mask);
    msg = cv_bridge::CvImage(hdr, "mono8", mask).toImageMsg();
    orange_mask_pub_.publish(msg);
}

void VisionNode::preProcessImage(cuda::GpuMat& gpu_img)
{

    // Convert to grayscale
    // cuda::cvtColor(gpu_img, gpu_img, COLOR_BGR2GRAY);
    applyTreshold(gpu_img);

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

    // Save edge detection result
    Mat cpu_edges;
    gpu_img.download(cpu_edges);
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(hdr, "mono8", cpu_edges).toImageMsg();
    edge_img_pub_.publish(msg);
}

void VisionNode::cropToROI(cuda::GpuMat& gpu_img)
{
    int width = gpu_img.cols;
    int height = gpu_img.rows;

    Mat roi_mask = Mat::zeros(height, width, CV_8UC1);

    // Define the region of interest as a polygon (example: trapezoid shape)
    std::vector<cv::Point> roi_points = {
        cv::Point(0, height),         // Bottom-left
        cv::Point(width, height),     // Bottom-right
        cv::Point(width, height / 5), // Top-right
        cv::Point(0, height / 5)      // Top-left
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

std::vector<Vec4i> VisionNode::getLines(cuda::GpuMat& gpu_img)
{
    auto min_line_length = this->get_parameter("min_line_length").as_int();
    auto max_detected_lines =
        this->get_parameter("max_detected_lines").as_int();
    auto max_line_gap = this->get_parameter("max_line_gap").as_int();
    auto rho = this->get_parameter("rho").as_double();

    cuda::GpuMat gpu_lines;
    auto line_detector = cuda::createHoughSegmentDetector(
        rho, CV_PI / 180.0f, min_line_length, max_line_gap, max_detected_lines);

    line_detector->detect(gpu_img, gpu_lines);

    // Download lines from GPU to CPU
    Mat lines_cpu;

    // Create the host matrix with CV_32SC4 type
    lines_cpu.create(1, gpu_lines.cols, CV_32SC4);
    gpu_lines.download(lines_cpu);

    // Convert to vector of Vec4i
    std::vector<Vec4i> lines;
    if (!lines_cpu.empty())
    {
        lines.reserve(lines_cpu.cols);
        int* data = lines_cpu.ptr<int>(); // Use int* instead of float*

        for (int i = 0; i < lines_cpu.cols; ++i)
        {
            // Each line is stored as 4 consecutive int values
            int x1 = data[i * 4];
            int y1 = data[i * 4 + 1];
            int x2 = data[i * 4 + 2];
            int y2 = data[i * 4 + 3];

            lines.push_back(Vec4i(x1, y1, x2, y2));
        }
    }

    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(),
                          DEBUG_LOG_FREQ_MS, "Detected %zu valid lines",
                          lines.size());
    return lines;
}

void VisionNode::drawLines(Mat& original_img, std::vector<Vec4i>& lines)
{
    for (const auto& line : lines)
    {
        auto slope =
            (line[3] - line[1]) / (line[2] - line[0]); //(y2 -y1) / (x2 - x1)
        auto color = slope < 0 ? Scalar(0, 255, 255) : Scalar(255, 255, 0);

        cv::line(original_img, Point(line[0], line[1]), Point(line[2], line[3]),
                 color, 2);
    }

    // Save and display results
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(hdr, "bgr8", original_img).toImageMsg();
    processed_img_pub_.publish(msg);
}

void VisionNode::publishLines(std::vector<cv::Vec4i>& lines)
{
    lane_msgs::msg::LanePositions msg;
    msg.header.stamp = this->now();

    std::vector<cv::Vec4i> left_lines, right_lines;
    for (const auto& line : lines)
    {
        float dx = line[2] - line[0];
        float dy = line[3] - line[1];

        if (std::abs(dx) < 1e-3)
            continue;

        float slope = dy / dx;
        float x_mid = (line[0] + line[2]) / 2.0f;

        if (slope < -0.1 && x_mid < (IMG_WIDTH / 2))
            left_lines.push_back(line);
        else if (slope > 0.1 && x_mid > (IMG_WIDTH / 2))
            right_lines.push_back(line);
    }

    for (auto& line : left_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.x = line[3];
        msg.left_lane.insert(msg.left_lane.end(), {p1, p2});
    }
    for (auto& line : right_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.x = line[3];
        msg.left_lane.insert(msg.left_lane.end(), {p1, p2});
    }

    lane_pos_pub_->publish(msg);
}
