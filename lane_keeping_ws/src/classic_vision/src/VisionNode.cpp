#include "VisionNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

VisionNode::VisionNode() : Node("vision_node")
{
    auto qos = rclcpp::QoS(60); // TODO: is QOS adapted?
    auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK)
        RCLCPP_ERROR(get_logger(), "Failed to set logging level to DEBUG");

    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::processImage(img); });

    this->declare_parameter("min_line_length", 100);
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
    Scalar lower_orange(10, 100, 100);
    Scalar upper_orange(25, 255, 255);

    // Apply mask. instead of creating a mask and masking the original img, we
    // use the mask as the picture for further processing
    cuda::inRange(gpu_img, lower_orange, upper_orange, gpu_img);
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

    // Draw lines
    Mat result = original_img.clone();
    for (const auto& line : lines)
    {
        cv::line(result, Point(line[0], line[1]), Point(line[2], line[3]),
                 Scalar(0, 255, 255), 2);
    }

    // Save and display results
    // imwrite("processed/result.jpg", result);
    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image::SharedPtr msg;
    msg = cv_bridge::CvImage(hdr, "bgr8", result).toImageMsg();
    processed_img_pub_.publish(msg);
}
