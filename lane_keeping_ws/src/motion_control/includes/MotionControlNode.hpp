#pragma once

#include <PIDController.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode();
        ~MotionControlNode() = default;

    private:
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr direction_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
        PIDController pid_controller_;

        void
        processLanePosition(lane_msgs::msg::LanePositions::SharedPtr lane_msg);

        void stopVehicle();
};
