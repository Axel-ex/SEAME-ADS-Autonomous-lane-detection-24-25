#include "CameraNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>

using namespace cv;

CameraNode::CameraNode() : Node("camera_node"), running_(true)
{
    cap_ = VideoCapture(0, cv::CAP_V4L2);
    if (!cap_.isOpened())
    {
        RCLCPP_ERROR(this->get_logger(), "Couldn't open the camera");
        return;
    }

    capture_thread_ = std::thread(&CameraNode::initPublisherAndCapture, this);
}

CameraNode::~CameraNode()
{
    running_ = false;
    if (capture_thread_.joinable())
        capture_thread_.join();
}

void CameraNode::initPublisherAndCapture()
{
    image_transport::ImageTransport it(shared_from_this());
    image_pub_ = it.advertise("image_raw", 1);

    RCLCPP_INFO(this->get_logger(), "Starting the camera");
    captureFrame();
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
        std_msgs::msg::Header hdr;
        sensor_msgs::msg::Image::SharedPtr msg;
        msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
        image_pub_.publish(msg);
        capture_rate_.sleep();
    }
}
