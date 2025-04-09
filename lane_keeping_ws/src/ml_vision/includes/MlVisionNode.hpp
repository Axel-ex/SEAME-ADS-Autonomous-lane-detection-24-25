#pragma once

#include <NvInfer.h>
#include <lane_msgs/msg/lane_positions.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class MlVisionNode : public rclcpp::Node
{
    public:
        MlVisionNode();
        ~MlVisionNode() = default;

    private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_sub_;
        rclcpp::Publisher<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_pub_;

        void rawImageCallback(sensor_msgs::msg::Image::SharedPtr img);
        nvinfer1::IExecutionContext* context_;
        cudaStream_t stream_;
        void* d_input_;
        void* d_output;
};
