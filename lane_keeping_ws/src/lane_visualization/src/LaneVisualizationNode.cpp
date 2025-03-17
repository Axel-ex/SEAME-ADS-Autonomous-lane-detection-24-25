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

    // lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
    //     "lane_positions", 10,
    //     [this](const lane_msgs::msg::LanePositions::SharedPtr msg)
    //     { this->processLanePosition(msg); });
    //
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
    // marker_pub_ =
    // this->create_publisher<visualization_msgs::msg::MarkerArray>(
    //     "lane_marks", 10);
}

void LaneVisualizationNode::initPublishers()
{
    auto it = image_transport::ImageTransport(shared_from_this());
    img_pub_ = it.advertise("processed_img", 1);
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
    if (left_coefs.empty() || right_coefs.empty())
    {
        RCLCPP_INFO(this->get_logger(), "empty polyfit coefs");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "starting analysis");
    auto converted = cv_bridge::toCvShare(msg, msg->encoding);
    cv::Mat img = converted->image;

    // Generate points from equations
    std::vector<cv::Point> left_poly, right_poly;
    for (int x = 0; x < 640; x++)
    {
        int y_left = ((left_coefs[2] * std::pow(x, 2)) + (left_coefs[1] * x) +
                      left_coefs[0]);
        int y_right = ((right_coefs[2] * std::pow(x, 2)) +
                       (right_coefs[1] * x) + right_coefs[0]);

        left_poly.emplace_back(x, y_left);
        right_poly.emplace_back(x, y_right);
    }

    cv::polylines(img, left_poly, false, cv::Scalar(0, 0, 255), 2);
    cv::polylines(img, right_poly, false, cv::Scalar(0, 255, 0), 2);

    for (auto& point : left_lane_pos_)
        cv::circle(img, cv::Point(point.x, point.y), 2, cv::Scalar(0, 255, 255),
                   2);
    for (auto& point : right_lane_pos_)
        cv::circle(img, cv::Point(point.x, point.y), 2, cv::Scalar(0, 0, 255),
                   2);

    cv_bridge::CvImage out_msg;
    out_msg.header = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image = img;
    RCLCPP_INFO(this->get_logger(), "publishing analysis");
    img_pub_.publish(out_msg.toImageMsg());
}

void LaneVisualizationNode::storeCoefs(
    const lane_msgs::msg::PolyfitCoefs::SharedPtr msg)
{
    left_coefs.clear();
    right_coefs.clear();

    for (auto& coef : msg->left_coefs)
        left_coefs.push_back(coef);
    for (auto& coef : msg->right_coefs)
        right_coefs.push_back(coef);
}

void LaneVisualizationNode::publishLaneMarks(const std::vector<Point32>& points,
                                             ColorRGBA& color)
{
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker center_mark;
    center_mark.header.frame_id = "map";
    center_mark.header.stamp = this->now();
    center_mark.id = 1;
    center_mark.type = visualization_msgs::msg::Marker::POINTS;
    center_mark.action = visualization_msgs::msg::Marker::ADD;
    center_mark.scale.x = 0.1; // Sphere size
    center_mark.scale.y = 0.1;
    center_mark.scale.z = 0.1;
    center_mark.color = color;

    for (const auto& point : points)
    {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;

        center_mark.points.push_back(p);
    }
    marker_array.markers.push_back(center_mark);
    marker_pub_->publish(marker_array);
}
