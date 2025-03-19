#include "PIDController.hpp"

PIDController::PIDController(double kp, double ki, double kd)
    : kp_(kp), ki_(ki), kd_(kd)
{
    prev_err_ = 0;
    integral_err_ = 0;
    last_time_ = std::chrono::steady_clock::now();
}

PIDController::PIDController() : kp_(0.5), ki_(0.1), kd_(0.2) {}

void PIDController::initializePID(double kp, double ki, double kd)
{
    kp_ = kp;
    kd_ = kd;
    ki_ = ki;
}

double PIDController::calculate(int error)
{
    double result;
    auto current_time = std::chrono::steady_clock::now();
    auto dt = current_time - last_time_;
    double dt_sec = dt.count();

    // Avoid division by zero or too small dt
    if (dt_sec < 0.001)
        dt_sec = 0.001;

    // Calculate integral and derivative terms
    integral_err_ += error * dt_sec;
    double derivative = (error - prev_err_) / dt_sec;

    // Calculate control signal
    double output = kp_ * error + ki_ * integral_err_ + kd_ * derivative;

    result = (kp_ * error);

    // Update values
    prev_err_ = error;
    last_time_ = std::chrono::steady_clock::now();

    return result;
}
