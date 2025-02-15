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

void VisionNode::processImage(sensor_msgs::msg::Image::SharedPtr img)
{
    try
    {
        RCLCPP_INFO(this->get_logger(), "Received image: %s",
                    img->encoding.c_str());

        auto converted = cv_bridge::toCvShare(img, img->encoding);
        if (converted->image.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Received an empty image!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Input image size: %dx%d",
                    converted->image.cols, converted->image.rows);

        // Upload the image to GPU
        cuda::GpuMat gpu_img;
        gpu_img.upload(converted->image);

        // Convert to grayscale
        cuda::GpuMat gpu_gray;
        cuda::cvtColor(gpu_img, gpu_gray, COLOR_BGR2GRAY);

        // Apply Gaussian blur
        cuda::GpuMat gpu_blurred;
        Ptr<cuda::Filter> gaussian_filter =
            cuda::createGaussianFilter(CV_8UC1, CV_8UC1, Size(3, 3), 1.0);
        gaussian_filter->apply(gpu_gray, gpu_blurred);

        // Apply Canny edge detection
        cuda::GpuMat gpu_edges;
        Ptr<cuda::CannyEdgeDetector> canny =
            cuda::createCannyEdgeDetector(30, 90);
        canny->detect(gpu_blurred, gpu_edges);

        // Save edge detection result
        Mat cpu_edges;
        gpu_edges.download(cpu_edges);
        imwrite("processed/edges.jpg", cpu_edges);
        RCLCPP_INFO(this->get_logger(),
                    "Edge image size: %dx%d, Non-zero pixels: %d",
                    cpu_edges.cols, cpu_edges.rows, countNonZero(cpu_edges));

        // Detect lines using Hough Transform
        cuda::GpuMat gpu_lines;
        auto line_detector =
            cuda::createHoughSegmentDetector(1.0f,           // rho
                                             CV_PI / 180.0f, // theta
                                             80,  // minimum line length
                                             20,  // maximum line gap
                                             1000 // maximum number of lines
            );

        line_detector->detect(gpu_edges, gpu_lines);

        // Download lines from GPU to CPU
        Mat lines_cpu;
        if (gpu_lines.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No lines detected on GPU");
            return;
        }

        // Create the host matrix with CV_32SC4 type
        lines_cpu.create(1, gpu_lines.cols, CV_32SC4);
        gpu_lines.download(lines_cpu);

        RCLCPP_INFO(this->get_logger(), "Lines matrix: type=%d, size=%dx%d",
                    lines_cpu.type(), lines_cpu.rows, lines_cpu.cols);

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

                // Print first few lines for debugging
                if (i < 5)
                {
                    RCLCPP_INFO(this->get_logger(),
                                "Line %d: (%d, %d) -> (%d, %d)", i, x1, y1, x2,
                                y2);
                }

                // Only add valid lines
                if (x1 >= 0 && x1 < converted->image.cols && y1 >= 0 &&
                    y1 < converted->image.rows && x2 >= 0 &&
                    x2 < converted->image.cols && y2 >= 0 &&
                    y2 < converted->image.rows)
                {
                    lines.push_back(
                        Vec4i(x1, y1, x2, y2)); // No conversion needed
                }
            }
        }

        // Draw lines
        Mat result = converted->image.clone();
        for (const auto& line : lines)
        {
            cv::line(result, Point(line[0], line[1]), Point(line[2], line[3]),
                     Scalar(0, 255, 255), 2);
        }

        // Save and display results
        imwrite("processed/result.jpg", result);

        RCLCPP_INFO(this->get_logger(), "Detected %zu valid lines",
                    lines.size());
    }
    catch (const cv::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error while processing the image: %s",
                     e.what());
    }
}
