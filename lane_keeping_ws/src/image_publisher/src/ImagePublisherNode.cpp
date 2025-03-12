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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5000),
                                     [this]() { publishImage(); });
    this->declare_parameter("image_name", "road_curve_piracer.jpg");
    RCLCPP_INFO(this->get_logger(), "%s initialized", this->get_name());
}

void ImagePublisherNode::publishImage()
{
    auto assets_path =
        "/home/axel/SEAME-ADS-Autonomous-lane-detection-24-25/assets/";
    auto file = this->get_parameter("image_name").as_string();
    auto full_name = assets_path + file;

    auto img = cv::imread(full_name, IMREAD_COLOR);
    if (img.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Couldnt open the image: %s",
                     full_name.c_str());
        return;
    }
    cv::resize(img, img, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);

    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "cam_frame";

    auto img_bridge =
        cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);

    sensor_msgs::msg::Image::SharedPtr img_msg = img_bridge.toImageMsg();
    image_pub_->publish(*img_msg);
}
