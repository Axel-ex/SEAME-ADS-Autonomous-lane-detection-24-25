#include "MotionControlNode.hpp"
#include "PolyFit.hpp"
#include <opencv2/opencv.hpp>

using namespace rclcpp;

/**
 * @brief Initializes ROS2 subscriptions, publishers, and control parameters.
 *
 * Parameters:
 * - `kp`, `ki`, `kd`: PID gains.
 * - `base_speed`: Default forward velocity.
 * - `lookahead_index`: Vertical pixel offset for lane center calculation.
 */
MotionControlNode::MotionControlNode()
    : Node("motion_control_node"), kalmman_filter_(0.1, 0.5), lane_buffer_(3)
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 1,
        [this](lane_msgs::msg::LanePositions::SharedPtr lane_msg)
        { MotionControlNode::lanePositionCallback(lane_msg); });
    polyfit_coefs_pub_ =
        create_publisher<lane_msgs::msg::PolyfitCoefs>("polyfit_coefs", 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    declare_parameter("kp", 1.0);
    declare_parameter("ki", 0.0);
    declare_parameter("kd", 0.0);
    declare_parameter("base_speed", 0.5);
    declare_parameter("lookahead_index", 130);
}

MotionControlNode::~MotionControlNode() { stopVehicle(); }

void MotionControlNode::initPIDController()
{
    pid_controller_.initializePID(shared_from_this());
}

/**
 * @brief Processes incoming lane positions and computes control commands.
 *
 * - Fits polynomials to lane markings (with buffer fallback for missing lanes).
 * - Computes lane center using lookahead distance.
 * - Applies Kalman filtering to lane center position.
 * - Calculates steering via PID control.
 *
 * @param lane_msg Shared pointer to LanePositions message.
 */
void MotionControlNode::lanePositionCallback(
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    std::vector<double> left_coefs, right_coefs;

    calculatePolyfitCoefs(left_coefs, right_coefs, lane_msg);
    lane_buffer_.addCoeffs(left_coefs, right_coefs);
    auto lane_center =
        findLaneCenter(left_coefs, right_coefs, lane_msg->image_height.data);

    if (!lane_center.x && !lane_center.y)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), WARN_FREQ,
                             "No lane points detected. Stoping the vehicle");
        stopVehicle();
        return;
    }

    lane_center.x = kalmman_filter_.update(lane_center.x);
    auto heading_point = findHeadingPoint(lane_msg->image_width.data,
                                          lane_msg->image_height.data);
    calculateAndPublishControls(lane_center, heading_point,
                                lane_msg->image_width.data);
    publishPolyfitCoefficients(left_coefs, right_coefs, lane_center);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_FREQ,
                         "right: %.4f; %.2f; %.2f, left: %.4f; %.2f; %.2f\n",
                         right_coefs[2], right_coefs[1], right_coefs[0],
                         left_coefs[2], left_coefs[1], left_coefs[0]);
}

/**
 * @brief Fits 2nd-degree polynomials to left/right lane points.
 *
 * - Separates and sorts lane coordinates by y-position.
 * - Uses buffered coefficients if current lane detection fails.
 * - Requires ≥3 points per lane for new fits.
 *
 * @param left_coefs Output vector for left lane coefficients [a, b, c]
 * (ax²+bx+c).
 * @param right_coefs Output vector for right lane coefficients.
 * @param lane_msg Input lane positions message.
 */
void MotionControlNode::calculatePolyfitCoefs(
    std::vector<double>& left_coefs, std::vector<double>& right_coefs,
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    std::vector<double> left_x, left_y, right_x, right_y;
    size_t degree = 2;

    separateAndOrderCoordinates(lane_msg->left_lane, left_x, left_y);
    separateAndOrderCoordinates(lane_msg->right_lane, right_x, right_y);

    if (left_x.size() >= 3 && right_x.size() >= 3)
    {
        left_coefs =
            calculate(left_x.data(), left_y.data(), degree, left_x.size());
        right_coefs =
            calculate(right_x.data(), right_y.data(), degree, right_x.size());
    }
    else if (left_x.size() < 3 && lane_buffer_.hasLeftLane())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_FREQ,
                             "Left lane missing → using buffered left lane");
        left_coefs = lane_buffer_.getLastLeft();
        right_coefs =
            calculate(right_x.data(), right_y.data(), degree, right_x.size());
    }
    else if (right_x.size() < 3 && lane_buffer_.hasRightLane())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_FREQ,
                             "Right lane missing → using buffered right lane");
        right_coefs = lane_buffer_.getLastRight();
        left_coefs =
            calculate(left_x.data(), left_y.data(), degree, left_x.size());
    }
    // If non of the condition are met, no lane are detected, the coefs stay
    // empty and the error is catch later in the program.
}

/**
 * @brief Computes lane center at a fixed lookahead distance.
 *
 * - Solves left/right lane polynomials at y = (image_height - lookahead_index).
 * - Returns (0,0) if no valid lanes detected.
 *
 * @param left_coefs Left lane polynomial coefficients.
 * @param right_coefs Right lane polynomial coefficients.
 * @param img_height Image height for coordinate scaling.
 * @return Lane center point (x,y) in image coordinates.
 */
Point32
MotionControlNode::findLaneCenter(const std::vector<double>& left_coefs,
                                  const std::vector<double>& right_coefs,
                                  int img_height)
{
    if (left_coefs.size() < 3 && right_coefs.size() < 3)
        return Point32();

    // choose a distance to look at
    int lookahead_index = get_parameter("lookahead_index").as_int();
    int lookahead = img_height - lookahead_index;
    lookahead = std::max(0, std::min(lookahead, img_height));

    double y = static_cast<double>(lookahead);
    double x_left =
        solveQuadratic(left_coefs[2], left_coefs[1], left_coefs[0] - y, false);
    double x_right = solveQuadratic(right_coefs[2], right_coefs[1],
                                    right_coefs[0] - y, true);

    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_FREQ,
                         "x_left: %.2f, x_right: %.2f", x_left, x_right);

    // Ensure x_left and x_right are valid (within image bounds)
    x_left = std::max(0.0, std::min(x_left, static_cast<double>(img_height)));
    x_right = std::max(0.0, std::min(x_right, static_cast<double>(img_height)));

    Point32 lane_center;
    lane_center.x = (x_left + x_right) / 2;
    lane_center.y = lookahead;

    return lane_center;
}

/**
 * @brief Find where the car is heading using lookahead_index parameter
 *
 * @param img_width
 * @param img_height
 * @return
 */
Point32 MotionControlNode::findHeadingPoint(int img_width, int img_height)
{
    Point32 result;

    auto lookahead_index = get_parameter("lookahead_index").as_int();
    int lookahead = img_height - lookahead_index;

    result.x = static_cast<double>(img_width) / 2;
    result.y = lookahead;
    return result;
}

/**
 * @brief Computes steering command using PID control.
 *
 * - Error: Normalized horizontal offset between lane center and image midpoint.
 * - Publishes Twist message with `base_speed` and steering angle.
 *
 * @param lane_center Current lane center position.
 * @param heading_point Reference point (image center at lookahead distance).
 * @param img_width For error normalization.
 */
void MotionControlNode::calculateAndPublishControls(Point32& lane_center,
                                                    Point32& heading_point,
                                                    int img_width)
{
    double error = heading_point.x - lane_center.x;
    error = error / (img_width / 2.0);

    double steering = pid_controller_.calculate(error);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), WARN_FREQ,
                         "lane_center: %.2f, error: %.2f, steering %.2f",
                         lane_center.x, error, steering);

    geometry_msgs::msg::Twist msg;
    msg.linear.x = get_parameter("base_speed").as_double();
    msg.angular.z = steering;
    cmd_vel_pub_->publish(msg);
}

/**
 * @brief Separates and sorts lane points by y-coordinate.
 *
 * - Converts vector<Point32> to sorted vectors of x/y coordinates.
 * - Ensures polynomial fitting follows lane direction.
 */
void MotionControlNode::separateAndOrderCoordinates(
    const std::vector<Point32>& points, std::vector<double>& x,
    std::vector<double>& y)
{
    x.clear();
    y.clear();

    std::vector<std::pair<double, double>> points_pair;
    for (auto& point : points)
        points_pair.push_back({point.y, point.x});
    std::sort(points_pair.begin(), points_pair.end());

    for (const auto& point : points_pair)
    {
        x.push_back(point.second);
        y.push_back(point.first);
    }
}

void MotionControlNode::stopVehicle()
{
    geometry_msgs::msg::Twist msg;

    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    cmd_vel_pub_->publish(msg);
}

/**
 * @brief Publishes polynomial coefficients and lane center.
 *
 * - Used for visualization and debugging.
 * - Converts coefficients to float for message compatibility.
 */
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
