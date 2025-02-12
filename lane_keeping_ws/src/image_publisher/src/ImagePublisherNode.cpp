#include "ImagePublisherNode.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

using namespace cv;

ImagePublisherNode::ImagePublisherNode() : Node("image_publisher_node")
{
    image_pub_ =
        this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(10),
                                     [this]() { publishImage(); });
    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void ImagePublisherNode::publishImage()
{
    auto file_name = "/home/jetpack/SEAME-ADS-Autonomous-lane-detection-24-25/"
                     "assets/road.jpg";
    auto img = cv::imread(file_name, IMREAD_COLOR);
    if (img.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Couldnt open the image");
        return;
    }

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "cam_frame";

    auto img_bridge =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);

    sensor_msgs::msg::Image::SharedPtr img_msg = img_bridge.toImageMsg();
    image_pub_->publish(*img_msg);
}
