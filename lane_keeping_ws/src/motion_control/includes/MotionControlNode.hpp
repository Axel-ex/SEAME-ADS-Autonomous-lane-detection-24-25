#pragma once

#include <rclcpp/rclcpp.hpp>

class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode();
        ~MotionControlNode() = default;
};
