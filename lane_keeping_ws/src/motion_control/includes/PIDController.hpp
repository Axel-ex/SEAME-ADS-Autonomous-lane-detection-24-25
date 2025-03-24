#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

class PIDController
{
    public:
        PIDController();
        ~PIDController() = default;

        void initializePID(std::shared_ptr<rclcpp::Node> node);
        double calculate(double error);

    private:
        // kp, ki, kd are parametrised
        std::shared_ptr<rclcpp::Node> node_ptr_;
        std::chrono::steady_clock::time_point last_time_;
        double prev_err_;
        double integral_err_;
};
