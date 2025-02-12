#include "VisionNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;

VisionNode::VisionNode() : Node("vision_node")
{
    auto qos = rclcpp::QoS(60);

    auto ret = rcutils_logging_set_logger_level(get_logger().get_name(),
                                                RCUTILS_LOG_SEVERITY_DEBUG);
    if (ret != RCUTILS_RET_OK)
        RCLCPP_ERROR(get_logger(), "Failed to set logging level to DEBUG");

    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::processImage(img); });

    // cv::namedWindow("processed");
    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

VisionNode::~VisionNode()
{
    // cv::destroyAllWindows();
}

void VisionNode::processImage(sensor_msgs::msg::Image::SharedPtr img)
{
    try
    {
        RCLCPP_INFO(this->get_logger(), "Received image: %s",
                    img->encoding.c_str());

        // Convert ROS image message to OpenCV Mat
        auto converted = cv_bridge::toCvShare(img, img->encoding);
        if (converted->image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received an empty image!");
            return;
        }

        // Upload the image to GPU
        cuda::GpuMat gpu_img;
        gpu_img.upload(converted->image);

        // Convert to grayscale
        cuda::GpuMat gpu_gray;
        cuda::cvtColor(gpu_img, gpu_gray, COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cuda::GpuMat gpu_blurred;
        Ptr<cuda::Filter> gaussian_filter =
            cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(3, 3), 1);
        gaussian_filter->apply(gpu_gray, gpu_blurred);

        // Edge detection
        // cuda::GpuMat gpu_edges;
        // auto canny = cuda::createCannyEdgeDetector(50, 150);
        // canny->detect(gpu_blurred, gpu_edges);

        // Detect lines using Hough Transform
        cuda::GpuMat gpu_lines;
        auto line_detector =
            cuda::createHoughSegmentDetector(1.0f,           // rho
                                             CV_PI / 180.0f, // theta
                                             50, // minimum line length
                                             5,  // maximum line gap
                                             300 // maximum number of lines
            );
        line_detector->detect(gpu_gray, gpu_lines);

        // Download lines from GPU to CPU
        Mat lines_cpu;
        gpu_lines.download(lines_cpu);

        // Convert to vector of Vec4i
        std::vector<Vec4i> lines;
        lines.reserve(lines_cpu.size().width);
        if (!lines_cpu.empty())
        {
            // Debug info
            RCLCPP_INFO(this->get_logger(), "Lines matrix size: %d x %d",
                        lines_cpu.rows, lines_cpu.cols);

            gpu_lines.row(0).download(Mat(lines).reshape(4, 1));
        }

        // Draw lines on a copy of the original image
        Mat result = converted->image.clone();
        for (const auto& line : lines)
        {
            cv::line(result, Point(line[0], line[1]), Point(line[2], line[3]),
                     Scalar(0, 255, 255), 2);
        }

        // Save intermediate results for debugging
        Mat cpu_gray, cpu_edges;
        gpu_gray.download(cpu_gray);
        // gpu_edges.download(cpu_edges);
        imwrite("processed/gray.jpg", cpu_gray);
        // imwrite("processed/edges.jpg", cpu_edges);
        imwrite("processed/result.jpg", result);

        RCLCPP_INFO(this->get_logger(), "Detected %zu lines", lines.size());
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while processing the image: %s",
                     e.what());
    }
}
