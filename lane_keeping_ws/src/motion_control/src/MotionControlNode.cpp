#include "MotionControlNode.hpp"
#include "PolyFit.hpp"

using namespace rclcpp;

MotionControlNode::MotionControlNode() : Node("motion_control_node")
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 10,
        [this](lane_msgs::msg::LanePositions::SharedPtr lane_msg)
        { MotionControlNode::lanePositionCallback(lane_msg); });
    polyfit_coefs_pub_ =
        create_publisher<lane_msgs::msg::PolyfitCoefs>("polyfit_coefs", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    declare_parameter("kp", 0.5);
    declare_parameter("ki", 0.1);
    declare_parameter("kd", 0.2);
    declare_parameter("base_speed", 0.2);
    declare_parameter("lookahead_index", 80);
    pid_controller_.initializePID(get_parameter("kp").as_double(),
                                  get_parameter("ki").as_double(),
                                  get_parameter("kd").as_double());
}

void MotionControlNode::lanePositionCallback(
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    std::vector<double> left_coefs, right_coefs;

    calculatePolyfitCoefs(left_coefs, right_coefs, lane_msg);
    Point32 lane_center =
        findLaneCenter(left_coefs, right_coefs, lane_msg->image_height.data);

    publishPolyfitCoefficients(left_coefs, right_coefs, lane_center);
}

void MotionControlNode::calculatePolyfitCoefs(
    std::vector<double>& left_coefs, std::vector<double>& right_coefs,
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    std::vector<double> left_x, left_y, right_x, right_y;
    size_t degree = 2;

    separateCoordinates(lane_msg->left_lane, left_x, left_y);
    separateCoordinates(lane_msg->right_lane, right_x, right_y);

    if (left_x.size() < 3 && right_x.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "No lane points detected. Stoping the vehicle");
        stopVehicle();
        return;
    }
    else if (left_x.size() < 3)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Lost left lane");
        right_coefs =
            calculate(right_x.data(), right_y.data(), degree, right_x.size());
        estimateMissingLane(left_coefs, right_coefs);
    }
    else if (right_x.size() < 3)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Lost right lane");
        left_coefs =
            calculate(left_x.data(), left_y.data(), degree, left_x.size());
        estimateMissingLane(left_coefs, right_coefs);
    }
    else
    {
        left_coefs =
            calculate(left_x.data(), left_y.data(), degree, left_x.size());
        right_coefs =
            calculate(right_x.data(), right_y.data(), degree, right_x.size());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                             "Estimated lane width: %d",
                             right_coefs[0] - left_coefs[0]);
    }
}

/**
 * @brief estimate the missing coefs lane coefs.
 *
 * estimate the missing lane coef by shifting the intercept in the right
 * direction depending on which lane is missing
 *
 * @param left_coefs
 * @param right_coefs
 */
void MotionControlNode::estimateMissingLane(std::vector<double>& left_coefs,
                                            std::vector<double>& right_coefs)
{
    int lane_width = 200;

    if (left_coefs.empty())
    {
        left_coefs = right_coefs;
        left_coefs[0] += lane_width;
    }
    else
    {
        right_coefs = left_coefs;
        right_coefs[0] -= lane_width;
    }
}

/**
 * @brief Find lane center at fix distance  img_height - "lookahead_index"
 *
 * @param left_coefs
 * @param right_coef
 * @return y position of the lane center at index "lookahead"
 */
Point32
MotionControlNode::findLaneCenter(const std::vector<double>& left_coefs,
                                  const std::vector<double>& right_coefs,
                                  int img_height)
{
    if (left_coefs.size() < 3 || right_coefs.size() < 3)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid polynomial coefficients. Expected 3 coefficients.");
        return Point32();
    }

    // choose a distance to look at
    int lookahead_index = get_parameter("lookahead_index").as_int();
    int lookahead = img_height - lookahead_index;
    lookahead = std::max(0, std::min(lookahead, img_height));

    // solve the quadtratic equations
    double y = static_cast<double>(lookahead);
    double x_left =
        solveQuadratic(left_coefs[2], left_coefs[1], left_coefs[0] - y);
    double x_right =
        solveQuadratic(right_coefs[2], right_coefs[1], right_coefs[0] - y);

    // Ensure x_left and x_right are valid (within image bounds)
    x_left = std::max(0.0, std::min(x_left, static_cast<double>(img_height)));
    x_right = std::max(0.0, std::min(x_right, static_cast<double>(img_height)));

    Point32 lane_center;
    lane_center.x = (x_left + x_right) / 2;
    lane_center.y = lookahead;

    return lane_center;
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

void MotionControlNode::publishPolyfitCoefficients(
    const std::vector<double>& left_coefs,
    const std::vector<double>& right_coefs, Point32& lane_center)
{
    lane_msgs::msg::PolyfitCoefs msg;

    msg.header.stamp = now();
    for (const auto& coef : left_coefs)
        msg.left_coefs.push_back(static_cast<float>(coef));
    for (const auto& coef : right_coefs)
        msg.right_coefs.push_back(static_cast<float>(coef));
    msg.lane_center = lane_center;
    polyfit_coefs_pub_->publish(msg);
}
