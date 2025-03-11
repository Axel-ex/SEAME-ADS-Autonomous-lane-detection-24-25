#include "MotionControlNode.hpp"

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

    // if its a straight line, the degree of poly is 1
    // but usually it will be 2
    // we can check variability of x to know
    size_t degree = 2;
    std::vector<double> left_coef =
        calculate(left_x.data(), left_y.data(), degree, left_x.size());
    std::vector<double> right_coef =
        calculate(right_x.data(), right_y.data(), degree, right_x.size());

    // assuming car always goes in straight line: dx**2 + ex + f = y, where d =
    // 0 intersection occurs on ax**2 + bx + c = dx**2 + ex + f therefor (a -
    // d)x**2 + (b - e)x + (c - f) = 0

    double d = 0; //?
    double e = 1; //?
    double f = 1; //?
    double left_col =
        quadraticFormula(left_coef[2], left_coef[1] - e, left_coef[1] - f);
    double right_col =
        quadraticFormula(right_coef[2], right_coef[1] - e, right_coef[1] - f);
}

double quadraticFormula(double a, double b, double c)
{
    double t_plus = (-1 * b + sqrt(b * b - 4 * a * c)) / (2 * a);
    double t_minu = (-1 * b - sqrt(b * b - 4 * a * c)) / (2 * a);
    if (t_plus >= 0 && (t_minu <= 0 || t_plus <= t_minu))
        return (t_plus);
    else if (t_minu >= 0)
        return (t_minu);
    else
        return (-1);
}

void separateCoordinates(const std::vector<Point32>& points,
                         std::vector<double>& x, std::vector<double>& y)
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

void MotionControlNode::stopVehicle()
{
    geometry_msgs::msg::Twist msg;

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velocity_pub_->publish(msg);
}
