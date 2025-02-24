#include "MotionControlNode.hpp"

using namespace rclcpp;

MotionControlNode::MotionControlNode() : Node("motion_control_node")
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 10,
        [this](lane_msgs::msg::LanePositions::SharedPtr lane_msg)
        { MotionControlNode::processLanePosition(lane_msg); });
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ =
        create_publisher<std_msgs::msg::String>("motion_control_status", 10);

    declare_parameter("kp", 0.5);
    declare_parameter("ki", 0.1);
    declare_parameter("kd", 0.2);
    declare_parameter("base_speed", 0.2);
    declare_parameter("lookahead_index", 10);
    declare_parameter("bucket_size", 10);
    pid_controller_.initializePID(get_parameter("kp").as_double(),
                                  get_parameter("ki").as_double(),
                                  get_parameter("kd").as_double());
}

void MotionControlNode::processLanePosition(
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    auto left_buckets = extractBuckets(lane_msg->left_lane);
    auto right_buckets = extractBuckets(lane_msg->right_lane);

    if (left_buckets.empty() || right_buckets.empty())
    {
        stopVehicle();
        return;
    }
    RCLCPP_INFO(get_logger(), "Found %d left buckets, %d right buckets",
                left_buckets.size(), right_buckets.size());

    auto left_lane_center = getBucketsAverage(left_buckets);
    auto right_lane_center = getBucketsAverage(right_buckets);

    // TODO:
    // Estimate position and steering action: could get an average of x position
    // for a bucket. then we could choose a bucket to look ahead to (target)
    // assess the distance from the center and steer accordingly
    // Integrate the error: The PID cotroller should be reducing the error at
    // each iteration.
}

std::vector<Point32>
MotionControlNode::getBucketsAverage(std::vector<std::vector<Point32>>& buckets)
{
    std::vector<Point32> result;

    for (auto& bucket : buckets)
    {
        Point32 point_avg;
        for (auto& point : bucket)
        {
            point_avg.x += point.x;
            point_avg.y += point.y;
        }
        point_avg.x /= bucket.size();
        point_avg.y /= bucket.size();
        result.push_back(point_avg);
    }

    return result;
}

std::vector<Point32>
MotionControlNode::filterLanePositions(std::vector<Point32>& points)
{
    return points;
}

/**
 * @brief Groups points into bucket based on the vertical spacing.
 *
 * @param points
 * @return
 */
std::vector<std::vector<Point32>>
MotionControlNode::extractBuckets(std::vector<Point32>& points)
{
    int bucket_size = get_parameter("bucket_size").as_int();
    std::vector<std::vector<Point32>> result;

    std::sort(points.begin(), points.end(),
              [](const Point32& a, const Point32& b) { return a.y < b.y; });

    std::vector<Point32> bucket;
    int upper_limit = points.front().y + bucket_size;
    for (const auto& point : points)
    {
        if (point.y > upper_limit)
        {
            result.push_back(std::move(bucket));
            bucket.clear();
            upper_limit += bucket_size;
        }
        bucket.push_back(point);
    }
    if (!bucket.empty())
        result.push_back(std::move(bucket));

    return result;
}

void MotionControlNode::stopVehicle()
{
    geometry_msgs::msg::Twist msg;

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velocity_pub_->publish(msg);
}
