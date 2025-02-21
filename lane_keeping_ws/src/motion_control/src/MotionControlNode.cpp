#include "MotionControlNode.hpp"
#include "eigen3/Eigen/Dense"

using namespace rclcpp;

MotionControlNode::MotionControlNode() : Node("motion_control_node")
{
    lane_pos_sub_ = this->create_subscription<lane_msgs::msg::LanePositions>(
        "lane_position", 10,
        [this](lane_msgs::msg::LanePositions::SharedPtr lane_msg)
        { MotionControlNode::processLanePosition(lane_msg); });
    direction_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_direction", 10);
    velocity_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    status_pub_ =
        create_publisher<std_msgs::msg::String>("motion_control_status", 10);

    declare_parameter("kp", 0.5);
    declare_parameter("ki", 0.1);
    declare_parameter("kd", 0.2);
    declare_parameter("base_speed", 0.2);
    declare_parameter("lookahead_index", 10);
    pid_controller_.initializePID(get_parameter("kp").as_double(),
                                  get_parameter("ki").as_double(),
                                  get_parameter("kd").as_double());
}

void MotionControlNode::processLanePosition(
    lane_msgs::msg::LanePositions::SharedPtr lane_msg)
{
    // TODO:
    // Filter out points
    // Estimate position and steering action
    // Integrate the error.
}
