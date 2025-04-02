#include "PIDController.hpp"
#include <algorithm>

PIDController::PIDController()
{
    prev_err_ = 0;
    integral_err_ = 0;
    last_time_ = std::chrono::steady_clock::now();
}

void PIDController::initializePID(std::shared_ptr<rclcpp::Node> node)
{
    node_ptr_ = node;
}

double PIDController::calculate(double error)
{
    auto current_time = std::chrono::steady_clock::now();
    auto dt = current_time - last_time_;
    double dt_sec = std::chrono::duration<double>(dt).count();
    auto kp = node_ptr_->get_parameter("kp").as_double();
    auto ki = node_ptr_->get_parameter("ki").as_double();
    auto kd = node_ptr_->get_parameter("kd").as_double();

    // Avoid division by zero or too small dt
    if (dt_sec < 0.001)
        dt_sec = 0.001;

    // Calculate integral and derivative terms
    integral_err_ += error * dt_sec;
    integral_err_ = std::clamp(integral_err_, -5.0, 5.0); // Avoid unwinding
    double derivative = (error - prev_err_) / dt_sec;

    // Calculate control signal
    double output = (kp * error) + (ki * integral_err_) + (kd * derivative);

    // Update values
    prev_err_ = error;
    last_time_ = current_time;

    // TODO: adjust using max steering
    // RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(),
    // 3000,
    //                      "integral: %.2f, derivative: %.2f", integral_err_,
    //                      derivative);
    //
    return output;
}
