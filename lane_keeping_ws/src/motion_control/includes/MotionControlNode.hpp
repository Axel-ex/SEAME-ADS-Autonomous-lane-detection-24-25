#pragma once

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
 * @brief Recieves lane positions and update direction accordingly.
 *
 * fit polyfits to right and left lane, asses lane center and drive the motor to
 * bring the vehicle closer to the lane center.
 *
 */
class MotionControlNode : public rclcpp::Node
{
    public:
        MotionControlNode();
        ~MotionControlNode() = default;
        void initPIDController();

    private:
        int estimated_lane_width_;
        PIDController pid_controller_;
        rclcpp::Subscription<lane_msgs::msg::LanePositions>::SharedPtr
            lane_pos_sub_;
        rclcpp::Publisher<lane_msgs::msg::PolyfitCoefs>::SharedPtr
            polyfit_coefs_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        void
        lanePositionCallback(lane_msgs::msg::LanePositions::SharedPtr lane_msg);

        void separateCoordinates(const std::vector<Point32>& points,
                                 std::vector<double>& x,
                                 std::vector<double>& y);
        void calculatePolyfitCoefs(
            std::vector<double>& left_coefs, std::vector<double>& right_coefs,
            lane_msgs::msg::LanePositions::SharedPtr lane_msg);
        Point32 findLaneCenter(const std::vector<double>& left_coef,
                               const std::vector<double>& right_coef,
                               int img_height);
        Point32 findHeadingPoint(int img_width, int img_height);
        void estimateMissingLane(std::vector<double>& left_coefs,
                                 std::vector<double>& right_coefs);
        void calculateAndPublishControls(Point32& lane_center,
                                         Point32& heading_point, int img_width);
        void publishPolyfitCoefficients(const std::vector<double>& left_coefs,
                                        const std::vector<double>& right_coefs,
                                        Point32& lane_center);
        void stopVehicle();
};
