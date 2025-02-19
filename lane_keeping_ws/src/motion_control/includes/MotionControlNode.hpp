#pragma once

#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>

class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode();
        ~MotionControlNode() = default;

    private:
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;

        void
        processLanePosition(lane_msgs::msg::LanePositions::SharedPtr lane_msg);
};
