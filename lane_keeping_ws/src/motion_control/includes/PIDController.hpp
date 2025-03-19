#pragma once

class PIDController
{
    public:
        PIDController(float kp, float ki, float kd);
        PIDController();
        ~PIDController() = default;

        void initializePID(float kp, float ki, float kd);
        double calculate(int error);

    private:
        float kp_;
        float ki_;
        float kd_;
};
