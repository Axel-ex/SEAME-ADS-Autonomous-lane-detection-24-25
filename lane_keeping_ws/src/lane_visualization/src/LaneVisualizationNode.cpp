#include "LaneVisualizationNode.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

LaneVisualizationNode::LaneVisualizationNode()
    : rclcpp::Node("lane_visualization_node")
{
    raw_img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg)
        { this->rawImageCallback(msg); });

    polyfit_coefs_sub_ =
        this->create_subscription<lane_msgs::msg::PolyfitCoefs>(
            "polyfit_coefs", 10,
            [this](const lane_msgs::msg::PolyfitCoefs::SharedPtr msg)
            { this->storeCoefs(msg); });

    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 10,
        [this](const lane_msgs::msg::LanePositions::SharedPtr msg)
        { this->storeLanePosition(msg); });

    RCLCPP_INFO(this->get_logger(), "starting LaneVisualisationNode");
}

void LaneVisualizationNode::initPublishers()
{
    auto it = image_transport::ImageTransport(shared_from_this());
    processed_img_pub_ = it.advertise("processed_img", 1);
}

void LaneVisualizationNode::storeLanePosition(
    const lane_msgs::msg::LanePositions::SharedPtr msg)
{
    left_lane_pos_.clear();
    right_lane_pos_.clear();

    for (auto& point : msg->left_lane)
        left_lane_pos_.push_back(point);
    for (auto& point : msg->right_lane)
        right_lane_pos_.push_back(point);
}

void LaneVisualizationNode::storeCoefs(
    const lane_msgs::msg::PolyfitCoefs::SharedPtr msg)
{
    left_coefs_.clear();
    right_coefs_.clear();

    for (auto& coef : msg->left_coefs)
        left_coefs_.push_back(coef);
    for (auto& coef : msg->right_coefs)
        right_coefs_.push_back(coef);
    lane_center_.x = msg->lane_center.x;
    lane_center_.y = msg->lane_center.y;
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
void LaneVisualizationNode::rawImageCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    if (left_coefs_.empty() || right_coefs_.empty())
    {
        RCLCPP_INFO(this->get_logger(), "empty polyfit coefs");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "starting analysis");
    auto converted = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat img = converted->image;

    // Generate points from equations
    std::vector<cv::Point> left_poly, right_poly;
    for (int x = 0; x < img.cols; x++)
    {
        int y_left = ((left_coefs_[2] * std::pow(x, 2)) + (left_coefs_[1] * x) +
                      left_coefs_[0]);
        int y_right = ((right_coefs_[2] * std::pow(x, 2)) +
                       (right_coefs_[1] * x) + right_coefs_[0]);

        left_poly.emplace_back(x, y_left);
        right_poly.emplace_back(x, y_right);
    }

    auto filterCondition = [img](const cv::Point& p)
    { return p.y < (img.rows * (1.0 / 3.0)); };

    // filter for point < 1/3 of the screen (cleaner representation)
    left_poly.erase(
        std::remove_if(left_poly.begin(), left_poly.end(), filterCondition),
        left_poly.end());
    right_poly.erase(
        std::remove_if(right_poly.begin(), right_poly.end(), filterCondition),
        right_poly.end());

    // Draw the polyfits
    cv::polylines(img, left_poly, false, cv::Scalar(0, 255, 255), 1);
    cv::polylines(img, right_poly, false, cv::Scalar(0, 0, 255), 1);

    // Draw lane points
    for (auto& point : left_lane_pos_)
        cv::circle(img, cv::Point(point.x, point.y), 1, cv::Scalar(0, 255, 255),
                   1);
    for (auto& point : right_lane_pos_)
        cv::circle(img, cv::Point(point.x, point.y), 1, cv::Scalar(0, 0, 255),
                   1);

    // Draw lane_center
    cv::circle(img, cv::Point(lane_center_.x, lane_center_.y), 1,
               cv::Scalar(0, 255, 0), 2);

    // Draw target point
    cv::circle(img, cv::Point(img.cols / 2, img.rows - 80), 1,
               cv::Scalar(255, 0, 0), 2);

    // publish the result
    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image = img;
    processed_img_pub_.publish(out_msg.toImageMsg());
}
