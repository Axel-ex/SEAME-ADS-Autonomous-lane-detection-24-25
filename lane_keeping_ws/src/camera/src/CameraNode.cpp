#include "CameraNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

CameraNode::CameraNode() : Node("camera_node"), running_(true)
{
    cap_ = VideoCapture(0);
    if (!cap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't open the camera");
        return;
    }
    this->create_publisher<sensor_msgs::msg::Image>("image_compressed",
                                                    rclcpp::QoS(60));
    capture_thread_ = std::thread(&CameraNode::captureFrame, this);
}
CameraNode::~CameraNode()
{
    running_ = false;
    if (capture_thread_.joinable())
        capture_thread_.join();
}

void CameraNode::captureFrame()
{
    Mat frame;

    while (rclcpp::ok() && running_)
    {
        cap_.read(frame);
        if (frame.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting frame");
            break;
        }

        // do stuff
    }
}
