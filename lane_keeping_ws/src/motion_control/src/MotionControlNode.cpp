#include "../includes/MotionControlNode.hpp"

using namespace rclcpp;

MotionControlNode::MotionControlNode() : Node("motion_control_node")
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 10,
        [this](lane_msgs::msg::LanePositions::SharedPtr lane_msg)
        { MotionControlNode::processLanePosition(lane_msg); });
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

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
    std::vector<double> left_x;
    std::vector<double> left_y;
    std::vector<double> right_x;
    std::vector<double> right_y;

    separateCoordinates(lane_msg->left_lane, left_x, left_y);
    separateCoordinates(lane_msg->right_lane, right_x, right_y);

    //if its a straight line, the degree of poly is 1
    //but usually it will be 2
    //we can check variability of x to know
    size_t degree = 2;
    std::vector<double> left_coef = calculate(left_x.data(), left_y.data(),
                                              degree, left_x.size());
    std::vector<double> right_coef = calculate(right_x.data(), right_y.data(),
                                               degree, right_x.size());
    //assuming car always goes in straight line: y = mx + b
    //intersection occurs on ax**2 + cx + d = mx + b
    //therefor ax**2 + (c - m)x + d - b = 0
    double m = 1; //?
    double b = 1; //?
    double left_col = quadraticFormula(left_coef[2], left_coef[1] - m,
                                       left_coef[1] - b);
    double right_col = quadraticFormula(right_coef[2], right_coef[1] - m,
                                        right_coef[1] - b);
}

double quadraticFormula(double a, double b, double c)
{
    double	t_plus = (-1 * b + sqrt(b * b - 4 * a * c)) / (2 * a);
	double	t_minu = (-1 * b - sqrt(b * b - 4 * a * c)) / (2 * a);
	if (t_plus >= 0 && (t_minu <= 0 || t_plus <= t_minu))
		return (t_plus);
	else if (t_minu >= 0)
		return (t_minu);
	else
		return (-1);
}

void separateCoordinates(const std::vector<Point32>& points, std::vector<double>& x, std::vector<double>& y)
{
    x.clear();
    y.clear();
    
    for (const auto& point : points)
    {
        x.push_back(point.x);
        y.push_back(point.y);
    }
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

    // Sort points by ascending y coordinate
    std::sort(points.begin(), points.end(),[](const Point32& a, const Point32& b) { return a.y < b.y; });

    // for each bucket, group them by group of size bucket_size
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
