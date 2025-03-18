#include "MotionControlNode.hpp"
#include "PolyFit.hpp"

using namespace rclcpp;

MotionControlNode::MotionControlNode() : Node("motion_control_node")
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 10,
        [this](lane_msgs::msg::LanePositions::SharedPtr lane_msg)
        { MotionControlNode::processLanePosition(lane_msg); });
    polyfit_coefs_pub_ =
        create_publisher<lane_msgs::msg::PolyfitCoefs>("polyfit_coefs", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    declare_parameter("kp", 0.5);
    declare_parameter("ki", 0.1);
    declare_parameter("kd", 0.2);
    declare_parameter("base_speed", 0.2);
    declare_parameter("lookahead_index", 80);
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

    size_t degree = 2;
    std::vector<double> left_coefs =
        calculate(left_x.data(), left_y.data(), degree, left_x.size());
    std::vector<double> right_coefs =
        calculate(right_x.data(), right_y.data(), degree, right_x.size());

    publishPolyfitCoefficients(left_coefs, right_coefs,
                               lane_msg->image_height.data);
}

void MotionControlNode::publishPolyfitCoefficients(
    const std::vector<double>& left_coefs,
    const std::vector<double>& right_coefs, int img_height)
{
    lane_msgs::msg::PolyfitCoefs msg;

    msg.header.stamp = now();
    for (const auto& coef : left_coefs)
        msg.left_coefs.push_back(static_cast<float>(coef));
    for (const auto& coef : right_coefs)
        msg.right_coefs.push_back(static_cast<float>(coef));
    msg.lane_center = findLaneCenter(left_coefs, right_coefs, img_height);

    polyfit_coefs_pub_->publish(msg);
}

/**
 * @brief Find lane center at fix distance "lookahead_index"
 *
 * @param left_coef
 * @param right_coef
 * @return y position of the lane center at index "lookahead"
 */
Point32 MotionControlNode::findLaneCenter(const std::vector<double>& left_coef,
                                          const std::vector<double>& right_coef,
                                          int img_height)
{
    int lookahead_index = get_parameter("lookahead_index").as_int();
    int lookahead = img_height - lookahead_index;

    // Ensure lookahead is within bounds
    lookahead = std::max(0, std::min(lookahead, img_height));

    double y = static_cast<double>(lookahead);

    double a_left = left_coef[2];
    double b_left = left_coef[1];
    double c_left = left_coef[0] - y;
    double x_left = quadraticFormula(a_left, b_left, c_left);

    double a_right = right_coef[2];
    double b_right = right_coef[1];
    double c_right = right_coef[0] - y;
    double x_right = quadraticFormula(a_right, b_right, c_right);

    // Ensure x_left and x_right are valid (within image bounds)
    x_left = std::max(0.0, std::min(x_left, static_cast<double>(img_height)));
    x_right = std::max(0.0, std::min(x_right, static_cast<double>(img_height)));

    Point32 lane_center;
    lane_center.x = (x_left + x_right) / 2;
    lane_center.y = lookahead;

    return lane_center;
}

// y = ax^2 + bx + c => ax^2 + bx + (c - y) = 0
double MotionControlNode::quadraticFormula(double a, double b, double c)
{
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0)
        return -1;

    double sqrt_discriminant = sqrt(discriminant);
    double t_plus = (-b + sqrt_discriminant) / (2 * a);
    double t_minu = (-b - sqrt_discriminant) / (2 * a);

    if (t_plus >= 0 && (t_minu <= 0 || t_plus <= t_minu))
        return t_plus;
    else if (t_minu >= 0)
        return t_minu;
    else
        return -1;
}

void MotionControlNode::separateCoordinates(const std::vector<Point32>& points,
                                            std::vector<double>& x,
                                            std::vector<double>& y)
{
    x.clear();
    y.clear();

    for (const auto& point : points)
    {
        x.push_back(point.x);
        y.push_back(point.y);
    }
}

void MotionControlNode::stopVehicle()
{
    geometry_msgs::msg::Twist msg;

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    velocity_pub_->publish(msg);
}
