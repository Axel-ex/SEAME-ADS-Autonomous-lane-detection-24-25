#pragma once

#include <KalmanFilter.hpp>
#include <LaneBuffer.hpp>
#include <PIDController.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lane_msgs/msg/lane_positions.hpp>
#include <lane_msgs/msg/polyfit_coefs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <std_msgs/msg/string.hpp>

using Point32 = geometry_msgs::msg::Point32;

/**
 * @class MotionControlNode
 * @brief Recieves lane positions, calculate lane center and steer to keep the
 * vehicle in the center of the lane.
 */
class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode();
        ~MotionControlNode();
        void initPIDController();

    private:
        LaneBuffer lane_buffer_;       // in case one of the lane is missing
        PIDController pid_controller_; // smooth out the steering
        KalmanFilter kalmman_filter_;  // filter absurd lane center measurements

        // ROS communication
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;
        rclcpp::Publisher<lane_msgs::msg::PolyfitCoefs>::SharedPtr
            polyfit_coefs_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        // Private methods
        void
        lanePositionCallback(lane_msgs::msg::LanePositions::SharedPtr lane_msg);
        void separateAndOrderCoordinates(const std::vector<Point32>& points,
                                         std::vector<double>& x,
                                         std::vector<double>& y);
        // void RANSACFilter(std::vector<double>& x, std::vector<double>& y);
        void calculatePolyfitCoefs(
            std::vector<double>& left_coefs, std::vector<double>& right_coefs,
            lane_msgs::msg::LanePositions::SharedPtr lane_msg);
        Point32 findLaneCenter(const std::vector<double>& left_coef,
                               const std::vector<double>& right_coef,
                               int img_height);
        Point32 findHeadingPoint(int img_width, int img_height);
        void calculateAndPublishControls(Point32& lane_center,
                                         Point32& heading_point, int img_width);
        void publishPolyfitCoefficients(const std::vector<double>& left_coefs,
                                        const std::vector<double>& right_coefs,
                                        Point32& lane_center);
        void stopVehicle();
};
