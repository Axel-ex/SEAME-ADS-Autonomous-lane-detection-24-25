#pragma once

class PIDController
{
    public:
        PIDController(float kp, float ki, float kd);
        ~PIDController();

    private:
        float kp_;
        float ki_;
        float kd_;
};
