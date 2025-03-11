#pragma once

#include <PIDController.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>

using Point32 = geometry_msgs::msg::Point32;

/**
 * @class MotionControlNode
 * @brief Recieves lane positions and update direction accordingly.
 *
 */
class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode();
        ~MotionControlNode() = default;

    private:
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
        PIDController pid_controller_;

        void
        processLanePosition(lane_msgs::msg::LanePositions::SharedPtr lane_msg);

        void stopVehicle();
        double quadraticFormula(double a, double b, double c);
        void separateCoordinates(const std::vector<Point32>& points,
                                 std::vector<double>& x,
                                 std::vector<double>& y);
};
