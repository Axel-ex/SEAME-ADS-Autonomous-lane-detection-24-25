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
    declare_parameter("lookahead_index", 10);
    declare_parameter("bucket_size", 10);
    pid_controller_.initializePID(get_parameter("kp").as_double(),
                                  get_parameter("ki").as_double(),
                                  get_parameter("kd").as_double());
}

void MotionControlNode::processLanePosition(
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    std::vector<double> x_cor;
    std::vector<double> y_cor;

    separateCoordinates(lane_msg->lane_points, x_cor, y_cor);

    std::vector<double> polyfit_coefs =
        calculate(x_cor.data(), y_cor.data(), 2, x_cor.size());

    publishPolyfitCoefficients(polyfit_coefs);
}

void MotionControlNode::publishPolyfitCoefficients(
    const std::vector<double>& polyfit_coefs)
{
    lane_msgs::msg::PolyfitCoefs msg;

    msg.header.stamp = now();
    for (const auto& coef : polyfit_coefs)
        msg.coefs.push_back(static_cast<float>(coef));

    polyfit_coefs_pub_->publish(msg);
}

double MotionControlNode::quadraticFormula(double a, double b, double c)
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
