#include "LaneVisualizationNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

LaneVisualizationNode::LaneVisualizationNode()
    : rclcpp::Node("lane_visualization_node")
{
    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { this->processImage(msg); });

    polyfit_coefs_sub_ =
        this->create_subscription<lane_msgs::msg::PolyfitCoefs>(
            "polyfit_coefs", 10,
            [this](const lane_msgs::msg::PolyfitCoefs::SharedPtr msg)
            { this->storeCoefs(msg); });

    RCLCPP_INFO(this->get_logger(), "starting LaneVisualisationNode");
}

void LaneVisualizationNode::initPublishers()
{
    auto it = image_transport::ImageTransport(shared_from_this());
    img_pub_ = it.advertise("processed_img", 1);
}

/**
 * @brief callback for raw image subscriber.
 *
 * Called upon receiving a new image. responsible for drawing all the features
 * extracted by our algorithms (lane points, polylines...) onto the original
 * image.
 *
 * @param msg
 */
void LaneVisualizationNode::processImage(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (polyfit_coefs.empty())
    {
        RCLCPP_WARN(this->get_logger(), "empty polyfit coefs");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "starting analysis");
    auto converted = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat img = converted->image;

    // Generate points from equations
    std::vector<cv::Point> polyfit_points;
    for (unsigned int x = 0; x < msg->height; x++)
    {
        int y = ((polyfit_coefs[2] * std::pow(x, 2)) + (polyfit_coefs[1] * x) +
                 polyfit_coefs[0]);

        polyfit_points.emplace_back(x, y);
    }

    cv::polylines(img, polyfit_points, false, cv::Scalar(0, 255, 0), 2);

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image = img;
    img_pub_.publish(out_msg.toImageMsg());
}

void LaneVisualizationNode::storeCoefs(
    const lane_msgs::msg::PolyfitCoefs::SharedPtr msg)
{
    polyfit_coefs.clear();

    for (auto coef : msg->coefs)
        polyfit_coefs.push_back(coef);
}
