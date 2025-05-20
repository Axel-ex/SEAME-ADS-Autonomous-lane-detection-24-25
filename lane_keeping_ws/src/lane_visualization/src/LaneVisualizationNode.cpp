#include "LaneVisualizationNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

LaneVisualizationNode::LaneVisualizationNode()
    : rclcpp::Node("lane_visualization_node")
{
    polyfit_coefs_sub_ =
        this->create_subscription<custom_msgs::msg::PolyfitCoefs>(
            "polyfit_coefs", 1,
            [this](const custom_msgs::msg::PolyfitCoefs::SharedPtr msg)
            { this->polyCoefsCallback(msg); });

    lane_pos_sub_ = this->create_subscription<custom_msgs::msg::LanePositions>(
        "lane_position", 1,
        [this](const custom_msgs::msg::LanePositions::SharedPtr msg)
        { this->storeLanePosition(msg); });

    RCLCPP_INFO(this->get_logger(), "starting LaneVisualisationNode");
}

void LaneVisualizationNode::initPublishers()
{
    auto it = image_transport::ImageTransport(shared_from_this());
    processed_img_pub_ = it.advertise("processed_img", 1);
}

void LaneVisualizationNode::storeLanePosition(
    const custom_msgs::msg::LanePositions::SharedPtr msg)
{
    left_lane_pos_.clear();
    right_lane_pos_.clear();

    for (auto& point : msg->left_lane)
        left_lane_pos_.push_back(point);
    for (auto& point : msg->right_lane)
        right_lane_pos_.push_back(point);
}

/**
 * @brief callback for raw image subscriber.
 *
 * Called upon receiving a new image. responsible for drawing all the features
 * extracted by our algorithms (lane points, polylines...) onto the original
 * image. publishes the result processed_img.
 *
 * @param msg
 */

void LaneVisualizationNode::polyCoefsCallback(
    const custom_msgs::msg::PolyfitCoefs::SharedPtr msg)
{
    left_coefs_.clear();
    right_coefs_.clear();

    for (auto& coef : msg->left_coefs)
        left_coefs_.push_back(coef);
    for (auto& coef : msg->right_coefs)
        right_coefs_.push_back(coef);
    lane_center_.x = msg->lane_center.x;
    lane_center_.y = msg->lane_center.y;

    if (left_coefs_.empty() || right_coefs_.empty())
        return;

    cv::Mat canvas(cv::Size(256, 256), CV_8UC3,
                   cv::Scalar(0, 0, 0)); // black canvas

    std::vector<cv::Point> left_poly, right_poly;
    for (int y = 0; y < canvas.rows; y++)
    {
        int x_left = static_cast<int>(left_coefs_[2] * std::pow(y, 2) +
                                      left_coefs_[1] * y + left_coefs_[0]);
        int x_right = static_cast<int>(right_coefs_[2] * std::pow(y, 2) +
                                       right_coefs_[1] * y + right_coefs_[0]);

        if (x_left >= 0 && x_left < canvas.cols)
            left_poly.emplace_back(x_left, y);
        if (x_right >= 0 && x_right < canvas.cols)
            right_poly.emplace_back(x_right, y);
    }
    // Draw lanes
    cv::polylines(canvas, left_poly, false, cv::Scalar(0, 255, 255), 2);
    cv::polylines(canvas, right_poly, false, cv::Scalar(0, 0, 255), 2);

    // Draw lane center
    cv::circle(canvas, cv::Point(lane_center_.x, lane_center_.y), 4,
               cv::Scalar(0, 255, 0), -1);

    // Draw target point (straight centerline reference)
    cv::circle(canvas, cv::Point(canvas.cols / 2, lane_center_.y), 4,
               cv::Scalar(255, 0, 0), -1);

    // Draw lane points
    for (auto& point : left_lane_pos_)
        cv::circle(canvas, cv::Point(point.x, point.y), 1,
                   cv::Scalar(0, 255, 255), 1);
    for (auto& point : right_lane_pos_)
        cv::circle(canvas, cv::Point(point.x, point.y), 1,
                   cv::Scalar(0, 0, 255), 1);
    // Publish
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = now();
    out_msg.encoding = "bgr8";
    out_msg.image = canvas;
    processed_img_pub_.publish(out_msg.toImageMsg());
}
