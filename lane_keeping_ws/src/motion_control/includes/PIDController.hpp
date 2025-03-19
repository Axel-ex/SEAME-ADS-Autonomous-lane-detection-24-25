#pragma once

#include <chrono>

class PIDController
{
    public:
        PIDController(double kp, double ki, double kd);
        PIDController();
        ~PIDController() = default;

        void initializePID(double kp, double ki, double kd);
        double calculate(int error);

    private:
        double kp_;
        double ki_;
        double kd_;

        double prev_err_;
        double integral_err_;
        std::chrono::steady_clock::time_point last_time_;
};
