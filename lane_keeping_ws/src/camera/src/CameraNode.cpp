#include "CameraNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/cudafilters.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/header.hpp>

using namespace cv;

CameraNode::CameraNode() : Node("camera_node"), running_(true)
{
    std::string pipeline =
        "nvarguscamerasrc ! video/x-raw(memory:NVMM), format=NV12, width=640, "
        "height=480, framerate=15/1 ! nvvidconv ! video/x-raw, format=BGRx ! "
        "videoconvert ! video/x-raw, format=BGR ! appsink";
    cap_ = VideoCapture(pipeline, cv::CAP_GSTREAMER);
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
    cap_.release();
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
    Mat frame(Size(1280, 720), CV_8UC3);

    while (rclcpp::ok() && running_)
    {
        if (cap_.read(frame)) // Non-blocking read
        {
            std_msgs::msg::Header hdr;
            sensor_msgs::msg::Image::SharedPtr msg;
            msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
            image_pub_.publish(msg);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Error getting frame");
            break;
        }
    }
}
