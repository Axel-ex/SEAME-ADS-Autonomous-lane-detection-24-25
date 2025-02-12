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

    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", qos,
        [this](sensor_msgs::msg::Image::SharedPtr img)
        { VisionNode::processImage(img); });

    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void VisionNode::processImage(sensor_msgs::msg::Image::SharedPtr img)
{
    cuda::GpuMat gpu_img;

    auto converted = cv_bridge::toCvShare(img, img->encoding);
    // imshow("received", converted->image);
    gpu_img.upload(converted->image);

    cuda::cvtColor(gpu_img, gpu_img, COLOR_BGR2GRAY);
    auto gaussian_filter =
        cuda::createGaussianFilter(CV_8UC1, CV_8UC1, {3, 3}, 1);
    gaussian_filter->apply(gpu_img, gpu_img);

    auto line_detector =
        cuda::createHoughSegmentDetector(1, CV_PI / 180, 50, 50, 10);
    cuda::GpuMat d_lines;
    line_detector->detect(gpu_img, d_lines);
    std::vector<Vec4i> lines;
    if (!d_lines.empty())
    {
        lines.resize(d_lines.cols);
        d_lines.download(Mat(lines));
    }
    for (auto& line : lines)
    {
        cv::line(converted->image, Point(line[0], line[1]),
                 Point(line[2], line[3]), Scalar(0, 255, 255), 2);
        imshow("detected lines", converted->image);
    }

    RCLCPP_INFO(this->get_logger(), "Image received");
}
