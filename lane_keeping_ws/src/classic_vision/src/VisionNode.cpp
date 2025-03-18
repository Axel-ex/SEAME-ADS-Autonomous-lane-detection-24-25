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
        { VisionNode::rawImageCallback(img); });
    lane_pos_pub_ = this->create_publisher<lane_msgs::msg::LanePositions>(
        "lane_position", 10);

    this->declare_parameter("min_line_length", 20);
    this->declare_parameter("max_line_gap", 20);
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
        publishLines(lines, gpu_img.rows);
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while processing the image: %s",
                     e.what());
    }
}

void VisionNode::preProcessImage(cuda::GpuMat& gpu_img)
{
    auto is_white_lane = get_parameter("is_white_lane").as_bool();

    applyTreshold(gpu_img, is_white_lane);
    applyMorphoTransfo(gpu_img);
    cropToROI(gpu_img);
    publishMaskImg(gpu_img);

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

void VisionNode::applyTreshold(cuda::GpuMat& gpu_img, bool is_white_lane)
{
    // H: hue, 0-179 color type
    // S: Saturation, 0-255 how vivid/pure
    // V: value, 0-255 how brght

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

void VisionNode::publishMaskImg(cuda::GpuMat& gpu_img)
{
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    Mat mask;
    gpu_img.download(mask);
    msg = cv_bridge::CvImage(hdr, "mono8", mask).toImageMsg();
    mask_pub_.publish(msg);
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

void VisionNode::publishLines(std::vector<cv::Vec4i>& lines, int img_width)
{
    lane_msgs::msg::LanePositions msg;
    msg.header.stamp = this->now();

    std::vector<cv::Vec4i> left_lines, right_lines;
    for (const auto& line : lines)
    {
        // float dx = line[2] - line[0];
        // float dy = line[3] - line[1];
        //
        // if (std::abs(dx) < 1e-3)
        //     continue;

        // float slope = dy / dx;
        float x_mid = (line[0] + line[2]) / 2.0f;

        if (x_mid < (static_cast<double>(img_width) / 2))
            left_lines.push_back(line);
        else if (x_mid > (static_cast<double>(img_width) / 2))
            right_lines.push_back(line);
    }

    for (auto& line : left_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.y = line[3];
        msg.left_lane.insert(msg.left_lane.end(), {p1, p2});
    }
    for (auto& line : right_lines)
    {
        geometry_msgs::msg::Point32 p1, p2;
        p1.x = line[0];
        p1.y = line[1];
        p2.x = line[2];
        p2.y = line[3];
        msg.right_lane.insert(msg.right_lane.end(), {p1, p2});
    }
    RCLCPP_INFO(this->get_logger(), "right_lanes size: %d\nleft_lane size: %d",
                right_lines.size(), left_lines.size());

    lane_pos_pub_->publish(msg);
}
